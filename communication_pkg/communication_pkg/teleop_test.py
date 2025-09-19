#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: ultima
"""
import rospy
import numpy as np 
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32

frequency = 30 # hz
min_pow = -0.5
max_pow = 0.5
max_time = 10.0

def talker():
    CAM_pub = rospy.Publisher(rospy.get_namespace()+'teleop/cam_pose', Quaternion, queue_size=1)
#    CAM_pub = rospy.Publisher(rospy.get_namespace()+'cam_pose', Quaternion, queue_size=1)
    Pedal_pub = rospy.Publisher(rospy.get_namespace()+'teleop/pedal_pose', Float32, queue_size=1)
    Wheel_pub = rospy.Publisher(rospy.get_namespace()+'teleop/wheel_pose', Float32, queue_size=1)
    rospy.init_node('TeleopTester', anonymous=True)
    rate = rospy.Rate(frequency) 

    val_cam = np.arange(0, np.pi, np.pi/frequency)
    size = max_time / (1.0/frequency)
    val_power = np.arange(min_pow, max_pow, (max_pow - min_pow)/size)
    t_cam = int(np.size(val_cam)/2)
    t_pow = int(np.size(val_power)/2)
    cam_pose = Quaternion()
   
    while not rospy.is_shutdown():
        cam_pose.w = 1
        cam_pose.x = val_cam[t_cam]
        cam_pose.y = val_cam[t_cam]
        cam_pose.z = val_cam[t_cam]   
                
        t_cam+=1
        if(t_cam >= np.size(val_cam)):
            t_cam = 0
            val_cam = np.flip(val_cam)
        
        t_pow+=1
        if(t_pow >= np.size(val_cam)):
            t_pow = 0
            val_power = np.flip(val_power)   
            
        CAM_pub.publish(cam_pose)
        Pedal_pub.publish(val_cam[t_pow])
        Wheel_pub.publish(val_power[t_cam])
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
