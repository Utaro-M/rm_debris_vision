#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import copy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import ModelCoefficientsArray
thre = 10000
polygons=None
def timer_cb(msg1,msg2,msg3):
    global polygons
    # print(msg1.header)
    # info_msg_polygon=msg2
    info_msg_polygon = PolygonArray()
    # info_msg_coefficient=msg3
    info_msg_coefficient = ModelCoefficientsArray()
    planes_indices = msg1.cluster_indices
    # print('len(planes_indices)', len(planes_indices))
    ind=0
    longest_ind=0
    longest_length=0
    if(len(planes_indices) >=1):
        for ind, plane in enumerate(planes_indices):
            length=len(plane.indices)
            if (length>longest_length):
                longest_ind=ind
                longest_length=length
                ind+=1
        # print(longest_ind)
        # # print (msg2.polygons)
        # print (len(msg2.polygons))
        # print (len(planes_indices))
        info_msg_polygon.polygons=msg2.polygons
        info_msg_polygon.header = msg2.header
        info_msg_coefficient.coefficients=[msg3.coefficients[longest_ind]]
        info_msg_coefficient.header = msg3.header
        pub_info_polygon.publish(info_msg_polygon)
        pub_info_coefficient.publish(info_msg_coefficient)
        # print('len(msg2.polygons)', len(msg2.polygons))
        polygons=msg2.polygons[longest_ind]
    if polygons is not None:
        pub_info_polygonstamped.publish(polygons)
    else:
        print("finding planes")

if __name__ == '__main__':
    rospy.init_node('get_plane')

    pub_info_polygon = rospy.Publisher('~output_polygon', PolygonArray, queue_size=1)
    pub_info_coefficient = rospy.Publisher('~output_coefficient', ModelCoefficientsArray, queue_size=1)
    pub_info_polygonstamped = rospy.Publisher('~focus_plane', PolygonStamped, queue_size=1)
    sub1 = message_filters.Subscriber("plane_extraction/plane_cluster_indices",ClusterPointIndices)
    sub2 = message_filters.Subscriber("plane_extraction/plane_polygons",PolygonArray)
    sub3 = message_filters.Subscriber("plane_extraction/plane_coefficients", ModelCoefficientsArray)
    # fps=10.0
    fps=10.0
    delay=1 / fps *0.5

    mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2,sub3],5,delay)
    # mf = message_filters.TimeSynchronizer([sub1,sub2,sub3], 10)
    mf.registerCallback(timer_cb)

    rospy.spin()
