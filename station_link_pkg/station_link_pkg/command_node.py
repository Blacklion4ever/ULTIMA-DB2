import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8MultiArray
from station_link_pkg.serial_manager import SerialManager

class CommandNode(Node):
    def __init__(self):
        super().__init__('station_command_node')
        self.serial = SerialManager()
        self.sub_pan_tilt = self.create_subscription(Int16MultiArray, '/rover/command/pan_tilt', self.cb_pan_tilt, 10)
        self.sub_steer = self.create_subscription(Int8MultiArray, '/rover/command/steering_propulsion', self.cb_steer, 10)

    def cb_pan_tilt(self, msg):
        payload = msg.data[0].to_bytes(2,'big',signed=True) + msg.data[1].to_bytes(2,'big',signed=True)
        self.serial.send_frame(0x01, 0x01, payload)

    def cb_steer(self, msg):
        payload = msg.data[0].to_bytes(1,'big',signed=True) + msg.data[1].to_bytes(1,'big',signed=True)
        self.serial.send_frame(0x01, 0x02, payload)

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()