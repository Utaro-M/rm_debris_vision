#!/usr/bin/env python
# coding:utf-8
import rospy
import message_filters
import numpy as np
import math
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray

def timer_cb(msg1):
    plarray=PolygonArray()
    plarray.header=msg1.header
    plarray.polygons=[msg1]
    pub_info_polygonstamped.publish(plarray)

if __name__ == '__main__':
    rospy.init_node('change_type')

    pub_info_polygonstamped = rospy.Publisher('~focus_plane', PolygonArray, queue_size=1)  
    sub1 = message_filters.Subscriber('get_plane/focus_plane', PolygonStamped)
    
    fps=10.0
    delay=1 / fps *0.5

    mf= message_filters.ApproximateTimeSynchronizer([sub1],5,delay)

    mf.registerCallback(timer_cb)

    rospy.spin()
