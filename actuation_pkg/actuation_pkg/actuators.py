#!/usr/bin/env python3.8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int16MultiArray

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

        # Souscriptions ROS2 vers topics station_link_pkg
        self.create_subscription(Int16MultiArray, '/rover/command/pan_tilt', self.set_cam_pose, 10)
        self.create_subscription(Int8MultiArray, '/rover/command/steering_propulsion', self.set_drive, 10)

    def home(self):
        # Direction neutre
        self.kit.servo[0].angle = 60
        # Camera tilt / pan neutre
        self.kit.servo[4].angle = 90
        self.kit.servo[5].angle = 90
        # Propulsion
        self.kit.continuous_servo[8].throttle = 0
        self.get_logger().info("Servos reset to home position")

    # -------- Caméra --------
    def set_cam_pose(self, msg: Int16MultiArray):
        # msg.data = [pan_deg, tilt_deg] -> -180 → +180
        pan_deg, tilt_deg = msg.data
        # Convertir en servo range (0-180°)
        pan_servo = np.clip(90 + pan_deg, 0, 180)
        tilt_servo = np.clip(90 + tilt_deg, 0, 180)
        self.kit.servo[5].angle = pan_servo
        self.kit.servo[4].angle = tilt_servo
        # self.get_logger().info(f"Camera -> Tilt: {tilt_servo:.2f}°, Pan: {pan_servo:.2f}°")

    # -------- Direction et propulsion --------
    def set_drive(self, msg: Int8MultiArray):
        # msg.data = [steering_deg, propulsion_percent]
        steering, propulsion = msg.data
        # Convertir steering en servo angle (15° → 160° sur ton servo)
        wheel_angle = np.clip(60 + steering, 15, 160)
        self.kit.servo[0].angle = wheel_angle
        # Convertir propulsion -100 → +100% en throttle -1 → +1
        throttle = np.clip(propulsion / 100.0, -1.0, 1.0)
        self.kit.continuous_servo[8].throttle = throttle
        # self.get_logger().info(f"Wheel: {wheel_angle:.2f}°, Throttle: {throttle:.2f}")

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
