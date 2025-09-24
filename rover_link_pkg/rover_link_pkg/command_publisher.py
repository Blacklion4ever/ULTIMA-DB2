#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8MultiArray


class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.pub_pan_tilt = self.create_publisher(Int16MultiArray, '/command/pan_tilt', 10)
        self.pub_steer = self.create_publisher(Int8MultiArray, '/command/steering_propulsion', 10)
        self.last_command_time = 0

    def publish_fallback(self,time_ns):
        """Publie des valeurs par défaut si aucune commande reçue récemment."""
        delta = abs(time_ns - self.last_command_time) * 1e-9
        if delta> 1.0:
            self.get_logger().info(f"no data recieved since {delta} sec, fallback")
            msg = Int16MultiArray()
            msg.data = [0, -45]
            self.pub_pan_tilt.publish(msg)
            msg2 = Int8MultiArray()
            msg2.data = [0, 0]
            self.pub_steer.publish(msg2)

    def publish_command(self, parsed_frame,time_ns):
        """Publie les commandes reçues depuis le lien série."""
        header = parsed_frame['header']
        sub = parsed_frame['subheader']
        payload = parsed_frame['payload']

        if header == 0x01:
            if sub == 0x01:  # Pan/Tilt
                pan = int.from_bytes(payload[0:2], 'big', signed=True)
                tilt = int.from_bytes(payload[2:4], 'big', signed=True)
                msg = Int16MultiArray()
                msg.data = [pan, tilt]
                self.pub_pan_tilt.publish(msg)
                self.get_logger().info(f"[CMD] Pan={pan}, Tilt={tilt}")
                self.last_command_time = time_ns

            elif sub == 0x02:  # Steering / Propulsion
                steer = int.from_bytes(payload[0:1], 'big', signed=True)
                prop = int.from_bytes(payload[1:2], 'big', signed=True)
                msg = Int8MultiArray()
                msg.data = [steer, prop]
                self.pub_steer.publish(msg)
                self.get_logger().info(f"[CMD] Steer={steer}, Propulsion={prop}")
                self.last_command_time = time_ns
