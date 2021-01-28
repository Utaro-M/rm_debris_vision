#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import copy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2

from pcl_msgs.msg import PointIndices
from std_msgs.msg import Float64, Bool ,Int32,Int32MultiArray
from rm_debris_vision.srv import CPthre

# threshould = 2000
threshould = 1000
threshould_l = 1000
threshould_r = 1000
length_l=0
length_r=0

def timer_cb(msg1,msg2):
    planes_indices_l = msg1.indices
    planes_indices_r = msg2.indices

    length_l=len(planes_indices_l)
    length_r=len(planes_indices_r)

    flag_list=Int32MultiArray()
    if length_l < threshould_l:
        flag_l= 0
    else:
        flag_l= 1
        
    if length_r < threshould_r:
        flag_r= 0
    else:
        flag_r= 1

    print('length_l', length_l)
    print('length_r', length_r)        
    print('flag_l', flag_l)
    print('flag_r', flag_r)    

    pub_num_l.publish(length_l)
    pub_num_r.publish(length_r)        
    # pub_drop_flag_l.publish(flag_l)
    # pub_drop_flag_r.publish(flag_r)
    flag_list.data=[flag_l,flag_r]
    pub_drop_flag.publish(flag_list)

def get_trigger (req):
    global threshould_l
    global threshould_r    
    threshould_l=req.larm.data
    threshould_r=req.rarm.data
    # print(req.trigger.data)
    print ("threshould larm set {}".format(threshould_l))
    
if __name__ == '__main__':
    rospy.init_node('count_points')

    pub_num_l = rospy.Publisher('/larm/num', Int32, queue_size=1)
    pub_num_r = rospy.Publisher('/rarm/num', Int32, queue_size=1)    
    # pub_drop_flag_l = rospy.Publisher('/drop_flag_larm', Bool, queue_size=1)
    # pub_drop_flag_r = rospy.Publisher('/drop_flag_rarm', Bool,  queue_size=1)
    pub_drop_flag = rospy.Publisher('/drop_flag_list', Int32MultiArray, queue_size=1)
    sub1 = message_filters.Subscriber("/in_hand_point/attention_clipper_larm/output/point_indices",PointIndices)
    sub2 = message_filters.Subscriber("/in_hand_point/attention_clipper_rarm/output/point_indices",PointIndices)

    fps=0.1
    delay=1 / fps *0.5

    s = rospy.Service('count_points_threshould',CPthre , get_trigger)
    
    mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2],2,delay)
    mf.registerCallback(timer_cb)

    rospy.spin()
