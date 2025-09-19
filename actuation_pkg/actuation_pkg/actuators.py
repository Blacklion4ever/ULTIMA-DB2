#!/usr/bin/env python3.8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion

import busio
import board
from adafruit_servokit import ServoKit
import numpy as np


class ActuatorsNode(Node):
    def __init__(self):
        super().__init__('actuators')

        # Initialisation du PCA9685
        self.get_logger().info("Initializing ServoKit...")
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.kit = ServoKit(channels=16, i2c=i2c_bus)
        self.get_logger().info("ServoKit Ready")

        # Remettre les servos à la position neutre
        self.home()

        # Souscriptions ROS2
        self.create_subscription(Quaternion, 'cam_pose', self.set_cam_pose, 10)
        self.create_subscription(Float64, 'wheel_pose', self.set_wheel_pose, 10)
        self.create_subscription(Float64, 'pedal_pose', self.set_pedal_pose, 10)

    def home(self):
        self.kit.servo[0].angle = 60  # Direction
        self.kit.servo[4].angle = 90  # Camera tilt
        self.kit.servo[5].angle = 90  # Camera pan
        self.kit.continuous_servo[8].throttle = 0
        self.get_logger().info("Servos reset to home position")

    def set_cam_pose(self, msg: Quaternion):
        # Tilt = Y, Pan = Z
        tilt = np.rad2deg(msg.y)
        pan = np.rad2deg(msg.z)
        self.kit.servo[4].angle = tilt
        self.kit.servo[5].angle = pan
        self.get_logger().info(f"Camera -> Tilt: {tilt:.2f}°, Pan: {pan:.2f}°")

    def set_wheel_pose(self, msg: Float64):
        val = max(min(np.rad2deg(msg.data), 160), 15)
        self.kit.servo[0].angle = val
        self.get_logger().info(f"Wheel -> {val:.2f}°")

    def set_pedal_pose(self, msg: Float64):
        val = max(min(msg.data, 0.25), -0.25)
        self.kit.continuous_servo[8].throttle = val
        self.get_logger().info(f"Throttle -> {val:.2f}")


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

