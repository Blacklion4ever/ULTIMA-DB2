#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Int8MultiArray
from std_srvs.srv import Trigger  # + service Trigger
import numpy as np

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_command_node')

        # Paramètre: index du bouton "stick gauche pressé" (L3).
        # Xbox/SDL par défaut: 9. Ajuste si besoin.
        self.declare_parameter('left_stick_button_index', 9)
        self.left_stick_button_index = int(self.get_parameter('left_stick_button_index').value)

        # Client service reset IMU
        self.reset_client = self.create_client(Trigger, '/reset_imu')
        self._warned_no_service = False
        self._prev_left_stick_btn = 0  # anti-rebond (front montant)

        # Publishers vers station_link_pkg
        self.pub_pan_tilt = self.create_publisher(Int16MultiArray, '/command/pan_tilt', 10)
        self.pub_steer_prop = self.create_publisher(Int8MultiArray, '/command/steering_propulsion', 10)

        # Dernier message Joy reçu
        self.last_joy_msg = None

        # Subscriber pour le joystick
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Timer pour publier à 50 Hz
        self.timer = self.create_timer(0.02, self.publish_loop)  # 50 Hz

        self.get_logger().info("Joystick node publishing to /rover/command topics at 50Hz.")

    def _on_reset_done(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"/reset_imu OK: {resp.message}")
            else:
                self.get_logger().warn(f"/reset_imu NOK: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Erreur appel /reset_imu: {e}")

    def _try_reset_imu(self):
        # Ne bloque pas; vérifie juste la disponibilité
        if not self.reset_client.wait_for_service(timeout_sec=0.0):
            if not self._warned_no_service:
                self.get_logger().warn("Service /reset_imu indisponible.")
                self._warned_no_service = True
            return
        req = Trigger.Request()
        future = self.reset_client.call_async(req)
        future.add_done_callback(self._on_reset_done)

    def joy_callback(self, msg: Joy):
        """Stocke l'état du joystick et gère le déclenchement L3."""
        self.last_joy_msg = msg

        # Lecture sécurisée du bouton L3
        cur = 0
        try:
            cur = int(msg.buttons[self.left_stick_button_index])
        except Exception:
            # Index invalide ou message incomplet
            if not self._warned_no_service:
                self.get_logger().warn(
                    f"Index bouton L3 invalide: {self.left_stick_button_index}"
                )
            return

        # Front montant: 0 -> 1
        if cur == 1 and self._prev_left_stick_btn == 0:
            self._try_reset_imu()

        self._prev_left_stick_btn = cur

    def publish_loop(self):
        if self.last_joy_msg is None:
            return

        msg = self.last_joy_msg

        # ---- 1. Pan/Tilt caméra (Int16MultiArray) ----
        pan_deg = int(msg.axes[3] * 1800)   # axes[3] = Stick droit horizontal
        tilt_deg = int(msg.axes[4] * 1800)  # axes[4] = Stick droit vertical

        pan_tilt_msg = Int16MultiArray()
        pan_tilt_msg.data = [pan_deg, tilt_deg]
        #self.pub_pan_tilt.publish(pan_tilt_msg)

        # ---- 2. Steering / Propulsion (Int8MultiArray) ----
        steering = int(-msg.axes[0] * 100)  # axes[0] = Stick gauche horizontal
        throttle = int((1 - msg.axes[5]) / 2 * 100)  # RT
        brake = int((1 - msg.axes[2]) / 2 * 100)     # LT
        propulsion = throttle - brake

        steer_prop_msg = Int8MultiArray()
        steer_prop_msg.data = [steering, propulsion]
        self.pub_steer_prop.publish(steer_prop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
