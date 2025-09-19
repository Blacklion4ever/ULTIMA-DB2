#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 30 20:11:24 2020
https://xbplib.readthedocs.io/en/latest/user_doc/discovering_the_xbee_network.html
@author: ultima
"""

from digi.xbee.devices import XBeeDevice
from digi.xbee.devices import TransmitException,TimeoutException

class Xbee():
    remote = None
    device = None
    
    def __init__(self):
        self.remote = None
        self.device = None
        self.frame = None
        self.callback= None
        
    def Recieve(self):
         return self.device.read_data_from( self.remote)
          
    def Start(self,port="/dev/ttyTHS1", baudrate=57600,remoteID="STATION", callback=None):
        self.device = XBeeDevice(port, baudrate)
        #open the local xbee 
        self.device.open()
        #Read the device information of the remote XBee device.
        self.device.read_device_info()
        # Add the callback.
        callback = callback
        if callback is not None:
            self.device.add_data_received_callback(callback)
        # get the network belonging
        xnet = self.device.get_network()
        # Discover the remote node whose node ID is ‘SOME NODE ID’.
        self.remote = xnet.discover_device(remoteID)
               
        if self.remote is None:
            print("stopping Xbee") 
            self.device.close()
            raise NameError("No remote device found")
        else:
            #Read the device information of the remote XBee device.
            self.remote.read_device_info()
            teleop_id = self.remote.get_node_id()
            print("Device with node id: %s found" % teleop_id)
    
    def Send(self,data):
        try:
            if self.remote is not None:
                self.device.send_data(self.remote, data)
                return True
            else:
                return False
        except (TransmitException,TimeoutException):
            print("Oops! XbeeSend failled")   
            return False
        
    def Stop(self):
        # Delete the callback
        if self.callback is not None:
            self.device.del_data_received_callback(self.callback)
        print("stopping Xbee")  
        self.device.close()


