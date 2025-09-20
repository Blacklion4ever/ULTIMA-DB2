#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 20:11:24 2020
@author: ultima
"""

import struct
from enum import IntEnum
import geometry_msgs.msg as msg
from std_msgs.msg import UInt64,Float32

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
    PING = 7       # <-- ajouté
    PONG = 8       # <-- ajouté
    TEST = 0xff

 
class FrameHandler:
    """
    Represents a frame of data to be sent to or which was received 
    from an XBee device
    [START_BYTE][FrameType][FrameSubType][FrameLabel][...FrameData...]
    """
    START_BYTE = 0x55
    
    def __init__(self,dataType = FrameType.UNKNOWN, dataSubType= FrameSubType.UNKNOWN, dataLabel= FrameLabel.UNKNOWN, data=None):
        self.frameType = dataType
        self.frameSubType = dataSubType 
        self.frameLabel = dataLabel
        #data is `BytesIO`
        self.frameData = data
        if self.frameData is not None:
            self.frameData.seek(0)
        self.isValid = False
        self.verify()
        
    def verify(self):
        self.isValid = self.frameType is not FrameType.UNKNOWN
        self.isValid = self.isValid and (self.frameSubType is not FrameSubType.UNKNOWN)
        self.isValid = self.isValid and (self.frameLabel is not FrameLabel.UNKNOWN)
   
    def isValid(self):
        return self.isValid
    
    def output(self):
        frame = bytes()        
        frame += struct.pack("B", self.START_BYTE)
        frame += struct.pack("B", self.frameType)
        frame += struct.pack("B", self.frameSubType)
        frame += struct.pack("B", self.frameLabel)         
        self.frameData.truncate(0x6C - len(frame))
        frame += self.frameData.readline() 
        saturated = 0x6C < len(frame)
        return frame,saturated
        
    def parse(self, FrameArray):
        self.frameType = FrameType.UNKNOWN
        self.frameSubType = FrameSubType.UNKNOWN 
        self.frameLabel = FrameLabel.UNKNOWN
        if len(FrameArray) < 4:
            ValueError("No avaiable data")
        # First byte is start byte
        if self.START_BYTE is not FrameArray[0]:
            raise ValueError("Invalid START_BYTE")
        self.frameType = FrameArray[1]
        self.frameSubType = FrameArray[2]
        self.frameLabel = FrameArray[3]
        self.verify()
        self.frameData = FrameArray[4:]
        
    def GetLabel(self):
        return self.frameLabel
    
    def GetStructuredData(self):     
        datasize = len(self.frameData)
        if self.isValid == False:
            return 0        
        if self.frameLabel == FrameLabel.CAM_POSE:            
            if datasize== 36:
                value = msg.Quaternion()
                #ignorer les 4 premiers octets header ROS2
                value.deserialize(self.frameData[4:])
                return value
            else:
                print("recieved : "+str(datasize)+" : expected 32")
                return None 
        elif self.frameLabel == FrameLabel.PEDAL_POSE:             
            if datasize== 4:
                value = Float32()
                value.deserialize(self.frameData)
                return value
            else:
                print("recieved : "+str(datasize)+" : expected 4")
                return None
        elif self.frameLabel == FrameLabel.WHEEL_POSE:
            if datasize== 4:
                value = Float32()
                value.deserialize(self.frameData)
                return value
            else:
                print("recieved : "+str(datasize)+" : expected 4")
                return None
        elif self.frameLabel == FrameLabel.TEST:             
            if datasize== 8:
                value = UInt64()
                value.deserialize(self.frameData)
                return value
            else:
                print("recieved : "+str(datasize)+" : expected 8")
                return None

        return None
