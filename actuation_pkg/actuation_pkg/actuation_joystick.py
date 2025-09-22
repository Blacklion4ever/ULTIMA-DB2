#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray, Int8MultiArray
import numpy as np

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_command_node')

        # Publishers vers station_link_pkg
        self.pub_pan_tilt = self.create_publisher(Int16MultiArray, '/rover/command/pan_tilt', 10)
        self.pub_steer_prop = self.create_publisher(Int8MultiArray, '/rover/command/steering_propulsion', 10)

        # Dernier message Joy reçu
        self.last_joy_msg = None

        # Subscriber pour le joystick
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Timer pour publier à 50 Hz
        self.timer = self.create_timer(0.02, self.publish_loop)  # 50 Hz

        self.get_logger().info("Joystick node publishing to /rover/command topics at 50Hz.")

    def joy_callback(self, msg: Joy):
        """Stocke simplement l'état du joystick."""
        self.last_joy_msg = msg

    def publish_loop(self):
        if self.last_joy_msg is None:
            return

        msg = self.last_joy_msg

        # ---- 1. Pan/Tilt caméra (Int16MultiArray) ----
        # Pan/Tilt caméra (Int16MultiArray)
        pan_deg = int(msg.axes[3] * 180)  # axes[3] = Stick droit horizontal, -1 → -180°, +1 → 180°
        tilt_deg = int(msg.axes[4] * 180)  # axes[4] = Stick droit vertical, -1 → -180°, +1 → 180°

        pan_tilt_msg = Int16MultiArray()
        pan_tilt_msg.data = [pan_deg, tilt_deg]
        self.pub_pan_tilt.publish(pan_tilt_msg)

        # ---- 2. Steering / Propulsion (Int8MultiArray) ----
        # axes[0] = Stick gauche horizontal (direction)
        steering = int(-msg.axes[0] * 100)  # -100 → +100°
        # Triggers: RT axes[5], LT axes[2], convert to -100 → +100%
        throttle = int((1 - msg.axes[5]) / 2 * 100)
        brake = int((1 - msg.axes[2]) / 2 * 100)
        propulsion = throttle - brake
        steer_prop_msg = Int8MultiArray()
        steer_prop_msg.data = [steering, propulsion]
        self.pub_steer_prop.publish(steer_prop_msg)

        # Optionnel : debug console
        # self.get_logger().info(
        #     f"Pan/Tilt: [{pan_deg}, {tilt_deg}] | Steering/Prop: [{steering}, {propulsion}]"
        # )

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
