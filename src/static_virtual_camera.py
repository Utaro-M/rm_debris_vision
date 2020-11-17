#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo
import tf
from tf2_msgs.msg import TFMessage
def timer_cb(msg):
    info_msg.header.stamp =  msg.transforms[0].header.stamp
    # info_msg.header.stamp = rospy.Time(0)  # - rospy.Duration(1.0)
    # print (info_msg.header.stamp)
    # print (rospy.Time.now())
    pub_info.publish(info_msg)

if __name__ == '__main__':
    rospy.init_node('static_virtual_camera')

    pub_info = rospy.Publisher('~camera_info', CameraInfo, queue_size=1)
    info_msg = CameraInfo()
    info_msg.header.frame_id = 'static_virtual_camera'

    # info_msg.height = 500
    # info_msg.width = 960
    info_msg.height = 480
    info_msg.width = 640
    # info_msg.height = 544
    # info_msg.width = 1024
    # info_msg.height = 1024
    # info_msg.width = 544
    info_msg.distortion_model = "rational_polynomial"
    fovx = 80
    fovy = 49
    # fovx = 49
    # fovy = 80
    # fovx = 100
    # fovy = 200

    fx = info_msg.width / 2.0 / np.tan(np.deg2rad(fovx / 2.0)) 
    fy = info_msg.height / 2.0 / np.tan(np.deg2rad(fovy / 2.0)) 
    cx = info_msg.width / 2.0 
    cy = info_msg.height / 2.0
    
    info_msg.D =     [8.236485481262207, -1.622314453125, -0.0007891572313383222, 0.0004136936622671783, 0.6135753989219666, 8.461249351501465, 0.02584407664835453, -0.5283768177032471]
    info_msg.K = np.array([fx, 0, cx,
                           0, fy, cy,
                           0, 0, 1.0])
    info_msg.P = np.array([fx, 0, cx, 0,
                           0, fy, cy, 0,
                           0, 0, 1, 0])
    info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    #optical frame
    # info_msg.height = 544
    # info_msg.width = 1024
    # info_msg.D =  [8.236485481262207, -1.622314453125, -0.0007891572313383222, 0.0004136936622671783, 0.6135753989219666, 8.461249351501465, 0.02584407664835453, -0.5283768177032471]
    # info_msg.K = [604.4480590820312, 0.0, 506.04071044921875, 0.0, 604.8341064453125, 253.6206817626953, 0.0, 0.0, 1.0]
    # info_msg.P = [558.8870239257812, 0.0, 512.0, 0.0, 0.0, 558.8870239257812, 272.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    # info_msg.R = [0.9999911785125732, -0.004174254834651947, -0.00047162213013507426, 0.004174285102635622, 0.9999912977218628, 6.258231587707996e-05, 0.0004713567905128002, -6.455044785980135e-05, 0.9999998807907104]

    
    rospy.Subscriber("/tf", TFMessage, timer_cb)        
    # rospy.Timer(rospy.Duration(1.0/30), timer_cb)
    rospy.spin()
