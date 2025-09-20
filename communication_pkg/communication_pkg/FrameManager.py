#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
from enum import IntEnum
import geometry_msgs.msg as msg
from std_msgs.msg import UInt64, Float32
import msgpack
from rosidl_runtime_py.convert import message_to_ordereddict
from io import BytesIO

# -------------------------------
# ENUMS
# -------------------------------
class FrameType(IntEnum):
    UNKNOWN = 0
    COMMAND = 1
    SIGNAL = 2
    TEST = 0xff

class FrameSubType(IntEnum):
    UNKNOWN = 0
    STATE = 1
    ACTUATION = 2
    PERCEPTION = 3
    TEST = 0xff
        
class FrameLabel(IntEnum):
    UNKNOWN = 0
    CAM_POSE = 1
    SPEED = 2
    IMU_ANGULAR_VEL = 3
    IMU_ACCEL = 4
    PEDAL_POSE = 5
    WHEEL_POSE = 6
    PING = 7
    PONG = 8
    TEST = 0xff

# -------------------------------
# Conversion dict -> ROS message (Eloquent)
# -------------------------------
def dict_to_msg(d, msg_type):
    """
    Convertit un dict (produit par message_to_ordereddict) en message ROS 2.
    Compatible ROS 2 Eloquent.
    """
    msg_obj = msg_type.__class__() if isinstance(msg_type, msg_type.__class__) else msg_type()
    for field, value in d.items():
        if hasattr(msg_obj, field):
            setattr(msg_obj, field, value)
    return msg_obj

# -------------------------------
# Sérialisation générique
# -------------------------------
def serialize_message(msg):
    """Sérialise n'importe quel message ROS 2 en bytes via msgpack"""
    msg_dict = message_to_ordereddict(msg)
    return msgpack.packb(msg_dict)

def deserialize_message(serialized_message, msg_type):
    """Désérialise un message ROS 2 à partir de bytes via msgpack"""
    msg_dict = msgpack.unpackb(serialized_message)
    return dict_to_msg(msg_dict, msg_type)

# -------------------------------
# Classe FrameHandler
# -------------------------------
class FrameHandler:
    START_BYTE = 0x55
    
    def __init__(self, dataType=FrameType.UNKNOWN, dataSubType=FrameSubType.UNKNOWN,
                 dataLabel=FrameLabel.UNKNOWN, data=None):
        self.frameType = dataType
        self.frameSubType = dataSubType 
        self.frameLabel = dataLabel
        self.frameData = BytesIO(data) if data else BytesIO()
        self.isValid = False
        self.verify()
        
    def verify(self):
        self.isValid = (self.frameType is not FrameType.UNKNOWN and
                        self.frameSubType is not FrameSubType.UNKNOWN and
                        self.frameLabel is not FrameLabel.UNKNOWN)
   
    def output(self):
        frame = bytes()
        frame += struct.pack("B", self.START_BYTE)
        frame += struct.pack("B", self.frameType)
        frame += struct.pack("B", self.frameSubType)
        frame += struct.pack("B", self.frameLabel)
        self.frameData.seek(0)
        data_bytes = self.frameData.read()
        frame += data_bytes[:0x6C - len(frame)]  # saturation à 108 octets
        saturated = 0x6C < len(frame)
        return frame, saturated
        
    def parse(self, FrameArray):
        if len(FrameArray) < 4:
            raise ValueError("No available data")
        if self.START_BYTE != FrameArray[0]:
            raise ValueError("Invalid START_BYTE")
        self.frameType = FrameArray[1]
        self.frameSubType = FrameArray[2]
        self.frameLabel = FrameArray[3]
        self.verify()
        self.frameData = BytesIO(FrameArray[4:])
        
    def GetLabel(self):
        return self.frameLabel
    
    def GetStructuredData(self):     
        self.frameData.seek(0)
        data_bytes = self.frameData.read()
        datasize = len(data_bytes)
        if not self.isValid:
            return None

        # Adapter la taille et le type
        if self.frameLabel == FrameLabel.CAM_POSE and datasize > 0:
            return deserialize_message(data_bytes, msg.Quaternion)
        elif self.frameLabel in [FrameLabel.PEDAL_POSE, FrameLabel.WHEEL_POSE] and datasize > 0:
            return deserialize_message(data_bytes, Float32())
        elif self.frameLabel == FrameLabel.TEST and datasize == 8:
            value = UInt64()
            value.data = struct.unpack("<Q", data_bytes)[0]
            return value
        else:
            print(f"Received {datasize} bytes: unexpected size for label {self.frameLabel}")
            return None

