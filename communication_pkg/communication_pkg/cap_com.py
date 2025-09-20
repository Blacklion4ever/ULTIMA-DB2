#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Eloquent compatible CapComNode
Ping envoyé uniquement par la station, Pong en réponse par le robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile
from time import time
from io import BytesIO
import msgpack
from rosidl_runtime_py.convert import message_to_ordereddict

# Imports relatifs
from .XbeeManager import Xbee
from .FrameManager import FrameHandler, FrameLabel, FrameSubType, FrameType

# -------------------------------------
# Sérialisation agnostique ROS2 (Eloquent compatible)
# -------------------------------------
def dict_to_msg(d, msg_type):
    """
    Convertit un dict (produit par message_to_ordereddict) en message ROS 2.
    Compatible ROS2 Eloquent.
    """
    msg_obj = msg_type.__class__() if isinstance(msg_type, msg_type.__class__) else msg_type()
    for field, value in d.items():
        if hasattr(msg_obj, field):
            setattr(msg_obj, field, value)
    return msg_obj

def serialize_message(msg):
    """Sérialise un message ROS 2 en bytes via msgpack"""
    msg_dict = message_to_ordereddict(msg)
    return msgpack.packb(msg_dict)

def deserialize_message(serialized_message, msg_type):
    """Désérialise un message ROS 2 à partir de bytes via msgpack"""
    msg_dict = msgpack.unpackb(serialized_message)
    return dict_to_msg(msg_dict, msg_type)

# -------------------------------------
# CapComNode
# -------------------------------------
class CapComNode(Node):
    def __init__(self, port='/dev/ttyTHS1', baudrate=57600, remoteID='STATION', mode='robot'):
        super().__init__('cap_com')

        # --- Déclarer les paramètres ROS2 ---
        self.port = self.declare_parameter('com_port', '/dev/ttyTHS1').get_parameter_value().string_value
        self.baudrate = self.declare_parameter('com_baudrate', 57600).get_parameter_value().integer_value
        self.remoteID = self.declare_parameter('remote_id', 'STATION').get_parameter_value().string_value
        self.mode = self.declare_parameter('mode', 'robot').get_parameter_value().string_value

        # Paramètres de supervision (station uniquement)
        self.ping_interval = self.declare_parameter('ping_interval', 1.0).get_parameter_value().double_value
        self.max_latency = self.declare_parameter('max_latency', 0.5).get_parameter_value().double_value
        self.ping_timeout = self.declare_parameter('ping_timeout', 2.0).get_parameter_value().double_value

        self.get_logger().info(f"Starting CapComNode in mode {self.mode}")
        self.get_logger().info(f"Port: {self.port}, Baudrate: {self.baudrate}, RemoteID: {self.remoteID}")

        self.qos = QoSProfile(depth=10)
        self.publisher_list = []

        # Souscriptions pour le mode téléop
        if self.mode == 'teleop':
            self.create_subscription(Quaternion, 'cam_pose', self.send_CAM_POSE, self.qos)
            self.create_subscription(Float32, 'pedal_pose', self.send_PEDAL_POSE, self.qos)
            self.create_subscription(Float32, 'wheel_pose', self.send_WHEEL_POSE, self.qos)

        # Gestion Xbee
        self.XbeeDevice = Xbee()
        self.XbeeDevice.Start(port=self.port, baudrate=self.baudrate, remoteID=self.remoteID, callback=self.Reception)

        # Supervision activée uniquement pour la station
        if self.mode == 'teleop':
            self.last_pong_time = time()
            self.last_ping_sent = None
            self.timer_ping = self.create_timer(self.ping_interval, self.send_ping)
            self.timer_watchdog = self.create_timer(1.0, self.check_connection)

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
        buffer = BytesIO(serialize_message(data))
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.CAM_POSE, buffer))

    def send_PEDAL_POSE(self, data):
        buffer = BytesIO(serialize_message(data))
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.PEDAL_POSE, buffer))

    def send_WHEEL_POSE(self, data):
        buffer = BytesIO(serialize_message(data))
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.WHEEL_POSE, buffer))

    # --- Ping / supervision ---
    def send_ping(self):
        if self.mode != 'teleop':
            return
        self.last_ping_sent = time()
        buffer = BytesIO()
        buffer.write(b"PING")
        buffer.seek(0)  # <-- IMPORTANT
        self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.PING, buffer))
        self.get_logger().info("Ping sent")

    def check_connection(self):
        if self.mode != 'teleop':
            return
        now = time()
        if now - self.last_pong_time > self.ping_timeout:
            self.get_logger().error("No pong received! Connection lost.")
            self.publish_diagnostic(None, connection_lost=True)

    # --- Reception ---
    def Reception(self, xbee_message):
        if xbee_message is not None:
            new_frame = FrameHandler()
            new_frame.parse(xbee_message.data)

            if new_frame.isValid:
                label = new_frame.GetLabel()

                # Ping / Pong
                if self.mode == 'robot' and label == FrameLabel.PING:
                    self.get_logger().info("Ping received -> sending Pong")
                    buffer = BytesIO()
                    buffer.write(b"PONG")
                    buffer.seek(0)
                    self.send_(FrameHandler(FrameType.COMMAND, FrameSubType.STATE, FrameLabel.PONG, buffer))
                    return

                if self.mode == 'teleop' and label == FrameLabel.PONG:
                    now = time()
                    latency = now - self.last_ping_sent if self.last_ping_sent else float('inf')
                    self.last_pong_time = now
                    self.get_logger().info(f"Pong received. Latency: {latency*1000:.2f} ms")
                    if latency > self.max_latency:
                        self.get_logger().warning(f"High latency detected: {latency*1000:.2f} ms")
                    self.publish_diagnostic(latency)
                    return

                # Données classiques
                pub = self.Publish(label)
                if pub is not None:
                    data = new_frame.GetStructuredData()
                    if data is not None and data != 0:
                        pub.publish(data)
                        self.get_logger().info("Publishing data received")

    # --- Diagnostic publication ---
    def publish_diagnostic(self, latency=None, connection_lost=False):
        diag_msg = DiagnosticArray()
        diag_status = DiagnosticStatus()
        diag_status.name = "CapComNode Xbee Link"
        diag_status.hardware_id = "Xbee"

        if connection_lost:
            diag_status.level = DiagnosticStatus.ERROR
            diag_status.message = "No pong received - connection lost"
        elif latency is not None:
            if latency > self.max_latency:
                diag_status.level = DiagnosticStatus.WARN
                diag_status.message = f"High latency: {latency*1000:.2f} ms"
            else:
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = "Link OK"
            diag_status.values.append(KeyValue(key="Latency (ms)", value=f"{latency*1000:.2f}"))

        diag_msg.status.append(diag_status)
        pub = self.get_or_create_publisher('diagnostics', DiagnosticArray)
        pub.publish(diag_msg)

    # --- Gestion des publishers dynamiques ---
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

# -------------------------------------
# Main
# -------------------------------------
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

