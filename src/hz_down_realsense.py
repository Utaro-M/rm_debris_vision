#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import copy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

msg1_save=None
msg2_save=None
msg3_save=None
def timer_cb(msg1,msg2,msg3):
    global msg1_save 
    global msg2_save 
    global msg3_save 
    msg1_save = msg1
    msg2_save = msg2
    msg3_save = msg3    
    
if __name__ == '__main__':
    rospy.init_node('hz_down_realsense')
    pub_info1 = rospy.Publisher('/rs_l515/color/camera_info/down', CameraInfo, queue_size=1)
    pub_info2 = rospy.Publisher('/rs_l515/color/image_rect_color/down', Image, queue_size=1)
    pub_info3 = rospy.Publisher('/rs_l515/aligned_depth_to_color/image_raw/down', Image, queue_size=1)

    sub1 = message_filters.Subscriber("/rs_l515/color/camera_info",CameraInfo)
    sub2 = message_filters.Subscriber("/rs_l515/color/image_rect_color", Image)
    sub3 = message_filters.Subscriber("/rs_l515/aligned_depth_to_color/image_raw", Image)    
    fps=10.0
    delay=1 / fps *0.5

    mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2,sub3],5,delay)
    mf.registerCallback(timer_cb)
    rate = rospy.get_param("~rate", 5)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        if msg1_save is not None:
            pub_info1.publish(msg1_save)
            pub_info2.publish(msg2_save)
            pub_info3.publish(msg3_save)
        else:
            print("Nothing")
        r.sleep()
