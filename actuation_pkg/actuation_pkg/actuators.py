#!/usr/bin/env python3.6
import os, time, numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int8MultiArray, Int16MultiArray

I2C_DEVICE = "/dev/i2c-1"
while not os.path.exists(I2C_DEVICE):
    print(f"Waiting for I2C device {I2C_DEVICE}...")
    time.sleep(1)

import busio, board
from adafruit_servokit import ServoKit

CMD_MAX_FORWARD = 0.60
CMD_MAX_REVERSE = -0.45

class ActuatorsNode(Node):
    def __init__(self):
        super().__init__('actuators')

        self.get_logger().info("Initializing ServoKit...")
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.kit = ServoKit(channels=16, i2c=i2c_bus)
        self.get_logger().info("ServoKit ready")

        # État propulsion
        self._u_cmd = 0.0
        self._y_out = 0.0
        self._eps = 1e-3
        self._timer_dt = 0.02       # 50 Hz
        self._T_smooth = 5.0        # 0→100% en 5 s
        self._T_medium = 2.5        # 0→100% en 2.5 s
        self._is_limiting = False

        # Paramètre drive_mode (Eloquent: set_parameters_callback)
        self.declare_parameter('drive_mode', 'smooth')
        self._mode = self.get_parameter('drive_mode').get_parameter_value().string_value
        self.set_parameters_callback(self._on_params_changed)
        self.get_logger().info(f"drive_mode initial={self._mode}")

        # Position neutre
        self.home()

        # Subscriptions
        self.create_subscription(Int16MultiArray, '/command/pan_tilt', self.set_cam_pose, 10)
        self.create_subscription(Int8MultiArray, '/command/steering_propulsion', self.set_drive, 10)

        # Boucle sortie fixe
        self.create_timer(self._timer_dt, self._update_throttle)

    # ----- Paramètres dynamiques (Eloquent) -----
    def _on_params_changed(self, params):
        ok = True
        for p in params:
            if p.name == 'drive_mode':
                val = p.value if isinstance(p.value, str) else str(p.value)
                if val not in ('smooth', 'medium', 'nolimit'):
                    ok = False
                    self.get_logger().warn(f"drive_mode invalide: {val}")
                else:
                    self._mode = val
                    self._is_limiting = False
                    self.get_logger().info(f"drive_mode={self._mode}")
        return SetParametersResult(successful=ok)

    # ----- Home -----
    def home(self):
        self.kit.servo[0].angle = 60
        self.kit.servo[4].angle = 90
        self.kit.servo[5].angle = 90
        self.kit.continuous_servo[8].throttle = 0.0
        self._u_cmd = 0.0
        self._y_out = 0.0

    # ----- Caméra -----
    def set_cam_pose(self, msg: Int16MultiArray):
        pan_deg, tilt_deg = msg.data
        pan_servo = float(np.clip(90 + pan_deg / 10.0, 0, 180))
        tilt_servo = float(np.clip(90 + tilt_deg / 10.0, 0, 180))
        self.kit.servo[5].angle = pan_servo
        self.kit.servo[4].angle = tilt_servo

    def map_throttle(self, user_pct):
        user_pct = np.clip(user_pct, -100.0, 100.0)
        if user_pct >= 0.0:
            return (user_pct / 100.0) * CMD_MAX_FORWARD
        else:
            return (user_pct / 100.0) * abs(CMD_MAX_REVERSE)


    # ----- Direction + consigne propulsion -----
    def set_drive(self, msg: Int8MultiArray):
        steering_deg, propulsion_percent = msg.data
        wheel_angle = float(np.clip(60 + steering_deg, 15, 160))
        self.kit.servo[0].angle = wheel_angle
        
        #self._u_cmd = float(np.clip(propulsion_percent / 100.0, -1.0, 1.0))
        self._u_cmd = self.map_throttle(propulsion_percent)
        self.get_logger().info(f"map_throttle in ({propulsion_percent:.3f}) out in ({self._u_cmd:.3f})")
       

    # ----- Boucle limiteur avec logs -----
    def _update_throttle(self):
        u = self._u_cmd
        y = self._y_out

        if self._mode == 'nolimit':
            if self._is_limiting:
                self.get_logger().info("limitation: arrêt (mode=nolimit)")
                self._is_limiting = False
            y_new = u
        else:
            T = self._T_smooth if self._mode == 'smooth' else self._T_medium
            dmax = self._timer_dt / T

            if abs(u) <= abs(y) + self._eps:
                if self._is_limiting:
                    self.get_logger().info(f"limitation: fin, |u| atteint ({abs(u):.3f})")
                    self._is_limiting = False
                y_new = u
            elif np.sign(u) != np.sign(y) and abs(y) > self._eps:
                self.get_logger().info(f"limitation: reset signe, u={u:.3f}, y={y:.3f} → 0.000")
                self._is_limiting = False
                y_new = 0.0
            else:
                y_allowed = min(abs(u), abs(y) + dmax)
                if y_allowed < abs(u) - 1e-9:
                    y_new = float(np.sign(u) * y_allowed)
                    if not self._is_limiting:
                        self.get_logger().info(
                            f"limitation: ramp [{self._mode}] |y| {abs(y):.3f} → {abs(y_new):.3f} "
                            f"(req {abs(u):.3f}), dmax {dmax:.4f}"
                        )
                    self._is_limiting = True
                else:
                    if self._is_limiting:
                        self.get_logger().info(f"limitation: fin, |u| atteint ({abs(u):.3f})")
                        self._is_limiting = False
                    y_new = u

        y_new = float(np.clip(y_new, -1.0, 1.0))
        if abs(y_new - y) > 1e-6:
            self.kit.continuous_servo[8].throttle = y_new
            self._y_out = y_new


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

