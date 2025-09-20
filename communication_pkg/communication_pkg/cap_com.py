#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Eloquent / Humble compatible CapComNode
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile

# Imports relatifs
from .XbeeManager import Xbee
from .FrameManager import FrameHandler, FrameLabel, FrameSubType, FrameType
from io import BytesIO


class CapComNode(Node):
    def __init__(self, port='/dev/ttyTHS1', baudrate=57600, remoteID='STATION', mode='robot'):
        super().__init__('cap_com')

        # --- Déclarer les paramètres ROS2 ---
        self.port = self.declare_parameter('com_port', '/dev/ttyTHS1').get_parameter_value().string_value
        self.baudrate = self.declare_parameter('com_baudrate', 57600).get_parameter_value().integer_value
        self.remoteID = self.declare_parameter('remote_id', 'STATION').get_parameter_value().string_value
        self.mode = self.declare_parameter('mode', 'robot').get_parameter_value().string_value

        self.get_logger().info(f"Starting CapComNode in mode {self.mode}")
        self.get_logger().info(f"Port: {self.port}, Baudrate: {self.baudrate}, RemoteID: {self.remoteID}")

        self.qos = QoSProfile(depth=10)
        self.publisher_list = []

        if self.mode == 'teleop':
            self.create_subscription(Quaternion, 'cam_pose', self.send_CAM_POSE, self.qos)
            self.create_subscription(Float32, 'pedal_pose', self.send_PEDAL_POSE, self.qos)
            self.create_subscription(Float32, 'wheel_pose', self.send_WHEEL_POSE, self.qos)

        self.XbeeDevice = Xbee()
        self.XbeeDevice.Start(port=self.port, baudrate=self.baudrate, remoteID=self.remoteID, callback=self.Reception)

    # --- Send / receive functions ---
    def send_(self, Frame):
        if Frame.isValid:
            toSend, troncated = Frame.output()
            if not troncated:
                self.XbeeDevice.Send(toSend)
            else:
                self.get_logger().warning("Data to be sent is too long")
        else:
            self.get_logger().warning("Not valid frame to be sent")

    def send_CAM_POSE(self, data):
        buffer = BytesIO()
        data.serialize(buffer)
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.CAM_POSE, buffer))

    def send_PEDAL_POSE(self, data):
        buffer = BytesIO()
        data.serialize(buffer)
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.PEDAL_POSE, buffer))

    def send_WHEEL_POSE(self, data):
        buffer = BytesIO()
        data.serialize(buffer)
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.WHEEL_POSE, buffer))

    def get_or_create_publisher(self, topic_name, msg_type):
        topic_name = self.get_namespace() + topic_name
        for name, pub in self.publisher_list:
            if name == topic_name:
                return pub
        pub = self.create_publisher(msg_type, topic_name, self.qos)
        self.publisher_list.append((topic_name, pub))
        self.get_logger().info(f"Creating new topic: {topic_name}")
        return pub

    def Publish(self, label):
        return {
            FrameLabel.CAM_POSE: self.get_or_create_publisher('cam_pose', Quaternion),
            FrameLabel.PEDAL_POSE: self.get_or_create_publisher('pedal_pose', Float32),
            FrameLabel.WHEEL_POSE: self.get_or_create_publisher('wheel_pose', Float32)
        }.get(label, None)

    def Reception(self, xbee_message):
        if xbee_message is not None:
            new_frame = FrameHandler()
            new_frame.parse(xbee_message.data)
            if new_frame.isValid:
                pub = self.Publish(new_frame.GetLabel())
                if pub is not None:
                    data = new_frame.GetStructuredData()
                    if data is not None and data != 0:
                        pub.publish(data)
                        self.get_logger().info("Publishing data recieved")


def main(args=None):
    rclpy.init(args=args)
    node = CapComNode()
    try:
        rclpy.spin(node)
    finally:
        node.XbeeDevice.Stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

