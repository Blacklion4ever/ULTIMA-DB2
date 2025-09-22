#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import SupervisionStatus
from station_link_pkg.serial_manager import SerialManager
import time


class SupervisionNode(Node):
    def __init__(self):
        super().__init__('station_supervision_node')
        self.serial = SerialManager()
        self.pub = self.create_publisher(SupervisionStatus, '/rover/status/supervision', 10)

        # Compteurs pour la supervision
        self.pings_sent = 0
        self.pongs_received = 0

        # Ping toutes les 250 ms
        self.timer = self.create_timer(0.25, self.send_ping)

    def send_ping(self):
        """Envoi d'un ping avec le timestamp actuel."""
        timestamp_ms = int(time.time() * 1000)  # timestamp en ms
        payload = timestamp_ms.to_bytes(8, 'big')  # 8 octets
        self.serial.send_frame(0x02, 0x01, payload)

        self.pings_sent += 1
        self.get_logger().info(f"[PING] Envoyé timestamp={timestamp_ms}")

        # Vérifie les réponses reçues
        self.check_responses()

    def check_responses(self):
        """Vérifie si des pongs sont reçus et publie l'état de la liaison."""
        frames = self.serial.read_frames()
        now_ms = int(time.time() * 1000)

        for f in frames:
            parsed = self.serial.fm.parse_frame(f)

            # Filtrer uniquement les réponses de supervision
            if parsed['header'] == 0x02 and parsed['subheader'] == 0x01:
                sent_timestamp = int.from_bytes(parsed['payload'], 'big')
                latency = now_ms - sent_timestamp  # Aller-retour complet (RTT)

                self.pongs_received += 1
                loss_pct = 100.0 * (self.pings_sent - self.pongs_received) / self.pings_sent if self.pings_sent > 0 else 0.0

                # Publier sur le topic
                msg = SupervisionStatus()
                msg.latency_ms = float(latency)
                msg.packet_loss_pct = float(loss_pct)
                msg.link_ok = True
                self.pub.publish(msg)

                self.get_logger().info(
                    f"[PONG] RTT={latency} ms | "
                    f"Loss={loss_pct:.1f}% | Sent={self.pings_sent} | Received={self.pongs_received}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = SupervisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
