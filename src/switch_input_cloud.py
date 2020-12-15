#!/usr/bin/env python

import rospy
import message_filters
from rm_debris_vision.srv import SwitchFlag
from sensor_msgs.msg import PointCloud2

points_multisense=None
points_l515=None
switch_flag =True
points=None
# def timer_cb(msg1,msg2):
#     global points_multisense
#     global points_l515
#     global switch_flag
#     points_multisense=msg1
#     points_l515=msg2
#     if switch_flag:
#         points=points_multisense
#     else:
#         points=points_l515
#     print ("OK")  
#     pub_info_input_cloud.publish(points)
    
def timer_cb1(msg):
    global points_multisense
    global switch_flag
    global points
    points_multisense=msg
    if switch_flag:
        points=points_multisense
    # print ("OK") 
    
    pub_info_input_cloud.publish(points)

def timer_cb2(msg):
    global points_l515
    global switch_flag
    global points
    points_l515=msg
    if (not switch_flag):
        points=points_l515
    print ("OK")    
    pub_info_input_cloud.publish(points)    

def get_trigger (req):
    global switch_flag
    switch_flag=req.trigger.data
    # print(req.trigger.data)
    if switch_flag:
        name="multisense"
    else:
        name="l515"
    print ("switch input cloud to {}".format(name))

if __name__ == '__main__':
    rospy.init_node('switch_input_cloud')

    pub_info_input_cloud = rospy.Publisher('/switched_cloud', PointCloud2, queue_size=1)
    s = rospy.Service('switch_pcl', SwitchFlag, get_trigger)
    
    rospy.Subscriber("/multisense_local/organized_image_points2_color",PointCloud2,timer_cb1)
    rospy.Subscriber("/rs_l515/depth_registered/points",PointCloud2,timer_cb2)
    
    # sub1 = message_filters.Subscriber("/multisense_local/organized_image_points2_color",PointCloud2)
    # sub2 = message_filters.Subscriber("/rs_l515/depth_registered/points",PointCloud2)
  
    # fps=0.2
    # delay=1 / fps *0.5
    # mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2],3,delay)
    # mf.registerCallback(timer_cb)

    rospy.spin()
