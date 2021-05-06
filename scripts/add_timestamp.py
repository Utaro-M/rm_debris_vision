#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import copy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

def timer_cb(msg1,msg2):
    info_msg = msg1
    info_msg.header.frame_id = 'with_timestamp_pcl2'
    info_msg.header.stamp =  msg2.header.stamp
    print(msg2.header.stamp)
    pub_info.publish(info_msg)

if __name__ == '__main__':
    rospy.init_node('add_timestamp')

    pub_info = rospy.Publisher('~with_timestamp_pcl2', PointCloud2, queue_size=1)

    sub1 = message_filters.Subscriber("depth_image_creator/output_cloud",PointCloud2)
    sub2 = message_filters.Subscriber("static_virtual_camera/camera_info", CameraInfo)
    
    fps=10.0
    delay=1 / fps *0.5

    mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2],5,delay)
    mf.registerCallback(timer_cb)

    rospy.spin()
