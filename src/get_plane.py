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
# def timer_cb(msg):
#     info_msg. =  msg.transforms.header.stamp
#     pub_info.publish(info_msg)


# if __name__ == '__main__':
#     rospy.init_node('static_virtual_camera')

#     pub_info = rospy.Publisher('~camera_inf', CameraInfo, queue_size=1)
#     info_msg = CameraInfo()
#     info_msg.header.frame_id = 'static_virtual_camera'
    
#     rospy.Subscriber("multi_plane_extraction/output",PointCloud2 , timer_cb)        
#     # rospy.Timer(rospy.Duration(1.0/30), timer_cb)
#     rospy.spin()nppn
thre = 10000
def timer_cb(msg1,msg2):
    # print(msg1.header)
    planes_indices = msg1.cluster_indices
    ind=0
    longest_ind=0
    longest_length=0
    if(len(planes_indices) >=1):
        for plane in planes_indices:
            length=len(plane.indices)
            # print ("loop{}".format(ind))
            # print(length)
            if (length>longest_length):
                longest_ind=ind
                longest_length=length
                ind+=1
                # info_msg.header=msg1.header
        # print(msg2.polygons[longest_ind].polygon.points[0])
        info_msg=msg2.polygons[longest_ind]
        pub_info.publish(info_msg)

if __name__ == '__main__':
    rospy.init_node('get_plane')

    pub_info = rospy.Publisher('~output', PolygonStamped, queue_size=1)
    info_msg=PolygonStamped()
    sub1 = message_filters.Subscriber("/organized_multi_plane_segmentation/output_refined",ClusterPointIndices)
    sub2 = message_filters.Subscriber("/organized_multi_plane_segmentation/output_refined_polygon",PolygonArray )
    
    fps=10.0
    delay=1 / fps *0.5

    mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2],5,delay)
    mf.registerCallback(timer_cb)

    rospy.spin()
