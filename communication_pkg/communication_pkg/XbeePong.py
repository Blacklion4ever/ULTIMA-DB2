#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 20:11:24 2020
https://xbplib.readthedocs.io/en/latest/user_doc/discovering_the_xbee_network.html
@author: ultima
"""
from XbeeManager import Xbee
import time
from FrameManager import FrameHandler,FrameLabel,FrameSubType,FrameType
from std_msgs.msg import UInt64
from io import BytesIO

def _send(Device, Frame):
    if Frame.isValid == True:
        toSend,troncated = Frame.output()
        if troncated is not True:
            Device.Send(toSend)
        else:
            print("Data to be send is too long") 
    else:
        print("Not valid frame to be sent")  

def _receive(xbee_message):
    global nb_recieved
    global nb_valid
    global delta
    if xbee_message is not None:
        nb_recieved +=1
        new_frame = FrameHandler()
        new_frame.parse(xbee_message.data)

        if new_frame.isValid == True:
            data = new_frame.GetStructuredData()
            if data != 0 and data is not None:
                index = Pong_list.index(data.data)
                timerx = int(time.time()*1e3)
                tranmited = Pong_list.pop(index)
                delta += (timerx - tranmited)
                nb_valid+=1   
                    
XbeeA = Xbee()
XbeeB = Xbee()

XbeeA.Start(baudrate=57600,callback = _receive)
XbeeB.Start(port="/dev/ttyUSB0",baudrate=57600,remoteID="ULTIMA",callback = _receive)

Pong_list=[]

nb_recieved = 0
nb_valid = 0
delta=0
max_send = 500
nb_sent=0
MasterTime = time.time()

while nb_recieved < max_send:

    if(nb_sent < max_send):
        ping = UInt64()
        ping.data = int(time.time()*1e3)
        Pong_list.append(ping.data)
        buffer = BytesIO()
        ping.serialize(buffer)
        _send(XbeeA,FrameHandler(FrameType.TEST,FrameSubType.TEST,FrameLabel.TEST,buffer))
        nb_sent+=1  
        

EndTime = time.time( )  
runtime = int((EndTime-MasterTime)*1e3)
print("Finished in: " + str(runtime) + " sec, estimate: " + str(int(runtime/max_send))+ " ms per transmit " )         
print("average delta time = "+ str(int(delta/nb_valid)) +" ms, with "+str(nb_valid)+" validated data") 
XbeeA.Stop()             
XbeeB.Stop()
