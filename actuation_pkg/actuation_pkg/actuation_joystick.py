#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
import numpy as np


class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('actuation_joystick')

        # Publishers
        self.pub_wheel = self.create_publisher(Float64, 'wheel_pose', 10)
        self.pub_cam = self.create_publisher(Quaternion, 'cam_pose', 10)
        self.pub_pedal = self.create_publisher(Float64, 'pedal_pose', 10)

        # Subscriber
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info("Joystick control node started.")

    def joy_callback(self, msg: Joy):
        """
        Mapping de la manette Xbox :
        - Stick gauche horizontal (axes[0]) -> Direction (wheel_pose)
        - Stick droit vertical (axes[4]) -> Camera Tilt (ch4)
        - Stick droit horizontal (axes[3]) -> Camera Pan (ch5)
        - Trigger RT (axes[5]) -> Accélérateur (pedal_pose)
        - Trigger LT (axes[2]) -> Frein ou marche arrière
        """

        # ---- 1. Direction (wheel_pose) ----
        # axes[0] = 1.0 (gauche) à -1.0 (droite)
        wheel_angle_rad = -msg.axes[0] * np.deg2rad(45)  # max ±45°
        wheel_msg = Float64()
        wheel_msg.data = wheel_angle_rad
        self.pub_wheel.publish(wheel_msg)

        # ---- 2. Camera tilt/pan (cam_pose) ----
        cam_msg = Quaternion()
        # Tilt (Y axis) = Stick droit vertical (axes[4]), inversé
        cam_msg.y = np.deg2rad(90 + msg.axes[4] * 45)  # 90° neutre
        # Pan (Z axis) = Stick droit horizontal (axes[3])
        cam_msg.z = np.deg2rad(90 + msg.axes[3] * 45)
        self.pub_cam.publish(cam_msg)

        # ---- 3. Moteur ESC (pedal_pose) ----
        # RT = axes[5], LT = axes[2]
        # Sur Xbox, triggers vont souvent de 1 (relâché) à -1 (pressé)
        throttle = (1 - msg.axes[5]) / 2.0 * 0.25  # plage 0 → 0.25
        brake = (1 - msg.axes[2]) / 2.0 * 0.25    # plage 0 → 0.25

        pedal_msg = Float64()
        pedal_msg.data = throttle - brake  # avant positif, arrière négatif
        self.pub_pedal.publish(pedal_msg)

        # Debug logs
        #self.get_logger().info(
        #    f"Wheel: {np.rad2deg(wheel_angle_rad):.1f}° | Tilt: {np.rad2deg(cam_msg.y):.1f}° | "
        #    f"Pan: {np.rad2deg(cam_msg.z):.1f}° | Throttle: {pedal_msg.data:.2f}"
        #)


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

