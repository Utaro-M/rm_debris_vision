#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo


def timer_cb(event):
    info_msg.header.stamp = rospy.Time.now()
    pub_info.publish(info_msg)


if __name__ == '__main__':
    rospy.init_node('static_virtual_camera')

    pub_info = rospy.Publisher('~camera_info', CameraInfo, queue_size=1)
    info_msg = CameraInfo()
    info_msg.header.frame_id = 'static_virtual_camera'

    info_msg.height = 640
    info_msg.width = 480
    fovx = 49
    fovy = 80
    # fovx = 100
    # fovy = 200    

    fx = info_msg.width / 2.0 / np.tan(np.deg2rad(fovx / 2.0)) 
    fy = info_msg.height / 2.0 / np.tan(np.deg2rad(fovy / 2.0)) 
    cx = info_msg.width / 2.0 
    cy = info_msg.height / 2.0 
    info_msg.D = [-0.071069, 0.024841, -6.7e-05, 0.024236, 0.0]
    info_msg.K = np.array([fx, 0, cx,
                           0, fy, cy,
                           0, 0, 1.0])
    info_msg.P = np.array([fx, 0, cx, 0,
                           0, fy, cy, 0,
                           0, 0, 1, 0])
    info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # info_msg.height = 480
    # info_msg.width = 640
    # fovx = 60
    # fovy = 49.5

    # info_msg.D = [-0.071069, 0.024841, -6.7e-05, 0.024236, 0.0]
    # info_msg.K = [496.734185, 0.0, 368.095466, 0.0, 491.721745, 248.578629, 0.0, 0.0, 1.0]
    # info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # info_msg.P = [471.903168, 0.0, 386.903326, 0.0, 0.0, 489.443268, 248.733955, 0.0, 0.0, 0.0, 1.0, 0.0]

    rospy.Timer(rospy.Duration(1.0/30), timer_cb)
    rospy.spin()
