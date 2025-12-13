#!/usr/bin/env python3.8
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int8MultiArray, Int16MultiArray

import time
import os
import numpy as np

# ----- Attente I2C -----
i2c_device = "/dev/i2c-1"
while not os.path.exists(i2c_device):
    print(f"Waiting for I2C device {i2c_device}...")
    time.sleep(1)

import busio
import board
from adafruit_servokit import ServoKit

class ActuatorsNode(Node):
    def __init__(self):
        super().__init__('actuators')

        # PCA9685
        self.get_logger().info("Initializing ServoKit...")
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.kit = ServoKit(channels=16, i2c=i2c_bus)
        self.get_logger().info("ServoKit Ready")

        # État propulsion
        self._u_cmd = 0.0     # consigne entrée [-1,1]
        self._y_out = 0.0     # sortie appliquée [-1,1]
        self._eps = 1e-3
        self._timer_dt = 0.02  # 50 Hz
        self._T_smooth = 5.0
        self._T_medium = 2.5

        # Paramètre de mode (smooth | medium | nolimit)
        self.declare_parameter('drive_mode', 'smooth')
        self._mode = self.get_parameter('drive_mode').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self._on_params_changed)

        # Position neutre
        self.home()

        # Souscriptions
        self.create_subscription(Int16MultiArray, '/command/pan_tilt', self.set_cam_pose, 10)
        self.create_subscription(Int8MultiArray, '/command/steering_propulsion', self.set_drive, 10)

        # Boucle de sortie à fréquence fixe
        self.create_timer(self._timer_dt, self._update_throttle)

    # ----- Paramètres -----
    def _on_params_changed(self, params):
        ok = True
        for p in params:
            if p.name == 'drive_mode':
                val = p.value
                if val not in ('smooth', 'medium', 'nolimit'):
                    ok = False
                else:
                    self._mode = val
                    self.get_logger().info(f"drive_mode={self._mode}")
        return SetParametersResult(successful=ok)

    # ----- Home -----
    def home(self):
        self.kit.servo[0].angle = 60      # direction neutre
        self.kit.servo[4].angle = 90      # caméra tilt
        self.kit.servo[5].angle = 90      # caméra pan
        self.kit.continuous_servo[8].throttle = 0.0
        self._u_cmd = 0.0
        self._y_out = 0.0

    # ----- Caméra -----
    def set_cam_pose(self, msg: Int16MultiArray):
        pan_deg, tilt_deg = msg.data  # [-180, +180]
        pan_servo = np.clip(90 + pan_deg/10.0, 0, 180)
        tilt_servo = np.clip(90 + tilt_deg/10.0, 0, 180)
        self.kit.servo[5].angle = pan_servo
        self.kit.servo[4].angle = tilt_servo

    # ----- Direction + consigne propulsion -----
    def set_drive(self, msg: Int8MultiArray):
        # msg.data = [steering_deg, propulsion_percent]
        steering_deg, propulsion_percent = msg.data

        # Direction: 60 + steering_deg, borné [15,160]
        wheel_angle = np.clip(60 + steering_deg, 15, 160)
        self.kit.servo[0].angle = float(wheel_angle)

        # Propulsion: -100..+100 -> -1..+1 (stockée comme consigne)
        u = np.clip(propulsion_percent / 100.0, -1.0, 1.0)
        self._u_cmd = float(u)

    # ----- Boucle limiteur -----
    def _update_throttle(self):
        u = self._u_cmd
        y = self._y_out

        if self._mode == 'nolimit':
            y_new = u
        else:
            # Δmax par tick selon le mode
            T = self._T_smooth if self._mode == 'smooth' else self._T_medium
            dmax = self._timer_dt / T  # montée 0→100% en T secondes

            # Décélérations libres ou stabilité
            if abs(u) <= abs(y) + self._eps:
                y_new = u
            # Changement de signe: chute libre à 0, puis rampe
            elif np.sign(u) != np.sign(y) and abs(y) > self._eps:
                y_new = 0.0
            else:
                # Limiter seulement les hausses d’amplitude
                y_mag = min(abs(u), abs(y) + dmax)
                y_new = float(np.sign(u) * y_mag)

        # Appliquer si variation significative, puis clip
        y_new = float(np.clip(y_new, -1.0, 1.0))
        if abs(y_new - y) > 1e-6:
            self.kit.continuous_servo[8].throttle = y_new
            self._y_out = y_new

    # ----- Main -----
def main(args=None):
    rclpy.init(args=args)
    node = ActuatorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.home()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
