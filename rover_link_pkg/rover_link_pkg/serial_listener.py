#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rover_link_pkg.command_publisher import CommandPublisher
from common_link_lib.frame_manager import FrameManager
import serial


class SerialListener(Node):
    def __init__(self):
        super().__init__('rover_serial_listener')

        # Ouverture du port série
        self.ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=0.01)
        self.fm = FrameManager()
        self.pub_command = CommandPublisher()

        self.get_logger().info("Rover Serial Listener démarré.")
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        """Lecture des données du port série et dispatch selon le type."""
        data = self.ser.read(64)
        if not data:
            return

        frames = self.fm.feed(data)
        for f in frames:
            parsed = self.fm.parse_frame(f)

            if parsed['header'] == 0x02 and parsed['subheader'] == 0x01:
                # Réponse Pong avec exactement le même payload
                pong_frame = self.fm.build_frame(0x02, 0x01, parsed['payload'])
                self.ser.write(pong_frame)
                timestamp = int.from_bytes(parsed['payload'], 'big')
                self.get_logger().info(f"[PONG] Timestamp renvoyé = {timestamp}")

            elif parsed['header'] == 0x01:
                # Commandes actuateurs
                self.pub_command.publish_command(parsed)


def main(args=None):
    rclpy.init(args=args)
    node = SerialListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
