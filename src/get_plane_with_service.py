#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import BoundingBoxArray
from rm_debris_vision.srv import GetPlane

polygons=None
centroid=[0, 0, 0]

def timer_cb(msg1,msg2,msg3):
    global polygons
    global centroid
    plane_idx=0
    if centroid != [0, 0, 0]:
        idx=0
        dif_tmp = 10000000000
        for i,box in enumerate(msg3.boxes):
            dif=math.sqrt((box.pose.position.x*1000 - centroid[0])**2+(box.pose.position.y*1000 - centroid[1])**2+(box.pose.position.z*1000 - centroid[2])**2)
            # print "dif ={}".format(dif)
            if(dif < dif_tmp):
                idx=i
                dif_tmp = dif
                plane_idx=msg3.boxes[idx].label
                # print "idx ={}".format(idx)
    else:
        planes_indices = msg1.cluster_indices
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
            plane_idx=longest_ind
    # print "plane_idx = {}".format(plane_idx)
    if(len(msg2.polygons) >=1):
    # if msg2.polygons is not None:
        polygons=msg2.polygons[plane_idx]
        pub_info_polygonstamped.publish(polygons)
    else:
        print("finding planes")

def get_trigger (req):
    global centroid
    centroid=[req.com.x,req.com.y,req.com.z]
    print ("\n\nset centroid {}\n".format(centroid))
    
if __name__ == '__main__':
    rospy.init_node('get_plane')

    pub_info_polygonstamped = rospy.Publisher('~focus_plane', PolygonStamped, queue_size=1)
    sub1 = message_filters.Subscriber("plane_extraction/plane_cluster_indices",ClusterPointIndices)
    sub2 = message_filters.Subscriber("plane_extraction/plane_polygons",PolygonArray)
    sub3 = message_filters.Subscriber("plane_extraction/plane_decomposer/boxes",BoundingBoxArray)
    s = rospy.Service('set_plane_centroid', GetPlane, get_trigger)
    
    fps=10.0
    delay=1 / fps *0.5

    mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2,sub3],5,delay)

    mf.registerCallback(timer_cb)

    rospy.spin()
