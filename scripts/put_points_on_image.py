#!/usr/bin/env python

import rospy
import numpy as np
import message_filters
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon #test
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rm_debris_vision.srv import GraspCandidates
from rm_debris_vision.srv import TriggerWithData

class PutPointsOnImage(object):
    def __init__(self):
        
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False
        self.frame_id = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        self.state = "sleep"
        self.flag=False
        self.u = 0
        self.v = 0
        self.u_v_list = []
        self.header_frame_id=""
        self.hands_interval=[100,300,500]
        self.cursor=0
        self.left_flag_count =[0, 0, 0, 0, 0]
        self.right_flag_count =[0, 0, 0, 0, 0]
        # publish
        self.pub_point_stamped = rospy.Publisher("~output", PointStamped, queue_size=1)
        self.pub_image = rospy.Publisher("~output/image", Image, queue_size=1)

        # subscribe
        rospy.Subscriber('~input/camera_info', CameraInfo, self.camera_info_cb)
        rospy.Subscriber("~input/image", Image, self.image_cb)
        #Joy
        sub1 = message_filters.Subscriber('~vive_left', Joy)
        sub2 = message_filters.Subscriber('~vive_right', Joy)
        self.mf= message_filters.ApproximateTimeSynchronizer([sub1,sub2],2,10)
        self.mf.registerCallback(self.joy_cb)

        self.s=rospy.Service("display_grasp_candidates", GraspCandidates,self.transform) #use service
        # rospy.Subscriber("~input/polygon_stamped", PolygonStamped, self.polygon_stamped_cb) #subscribe points
        # rospy.Subscriber("~input/point_stamped", PointStamped, self.point_stamped_cb)
        
    def camera_info_cb(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.frame_id = msg.header.frame_id
        self.is_camera_arrived = True

    # detect button and change state
    # state :: sleep -> button input (change mode)-> waiting (wait for operator instruction) -> button input (operator chooses target candidates) -> sleep (advertise choosed candidates and request ik solving)
    def joy_cb(self,left_msg,right_msg):
        self.set_button_state(left_msg,right_msg)
        
        if (self.state == "sleep"):
            if self.pushed(left,3) and self.pushed(right,3): # buttons: [0, 0, 0, 0, 1] and [0, 0, 0, 0, 1] state: sleep -> waiting
                self.send_trigger(True,self.hands_interval)
                self.state = "waiting"
            if self.pushed(left,2) and self.pushed(right,2):
                self.send_trigger(True,'auto_ik_trigger') #first: set start pos
                self.state = "getting_ref"
        elif(self.state == "getting_ref"):
            if self.pushed(left,0): #reset
                self.state = "sleep"
            if self.pushed(left,2) and self.pushed(right,2):
                self.send_trigger(True,'auto_ik_trigger') #second: set goal pos
                self.state = "sleep"

        elif(self.state == "waiting"):
            if self.pushed(left,0): #reset
                self.state = "sleep"
            if self.pushed(left,2):  # buttons: [0, 0, 0, 1, 0] :: change candidates
                self.cursor +=1
            elif self.pushed(right,2): # buttons: [0, 0, 0, 1, 0] :: change candidates
                self.cursor -=1
            if(self.cursor < 0):
                self.cursor = 0
                # self.cursor = len(self.u_v_list)/2 -1
            if(len(self.u_v_list)/2 <= self.cursor):
                self.cursor = len(self.u_v_list)/2 -1
                # self.cursor = 0
                
            if self.pushed(left,3) and self.pushed(right,3): # buttons: [1, 0, 0, 0, 0] and [1, 0, 0, 0, 0] state: waiting -> sleep
                self.send_trigger(False,[self.cursor])
                self.state = "sleep"
            

        rospy.wait_for_service('auto_ik_trigger')
            
    def set_button_state(self,left,right):
        for i in range (len(left_msg.buttons)):
            if left_msg.buttons[i]==1:
                self.left_flag_count[i] += 1
            else:
                self.left_flag_count[i] = 0
                
            if right_msg.buttons[i]==1:
                self.right_flag_count[i] += 1
            else:
                self.right_flag_count[i] = 0
                
    def pushed(self,hand,button_num,thre=None):
        if (not thre):
            thre=self.count_threshould
        if(hand =="left"):
            if self.left_flag_count[button_num] > thre:
                return True
            else:
                return False
        else:
            if self.right_flag_count[button_num] > thre:
                return True
            else:
                return False
            
    # request candidates info or request solve ik and move robot
    def send_trigger(self,flag,data,svc_name='joy_trigger'):
        rospy.wait_for_service(svc_name)
        try:
            Trigger_with_data = rospy.ServiceProxy(svc_name, TriggerWithData)
            resp1 = Trigger_with_data(flag,data)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # when state==waiting : add circles to image and publish it
    # when state == otherwise : only publish original image
    def image_cb(self,msg):
        self.header_frame_id = msg.header.frame_id
        if self.state == "waiting" :
            print "in image_cb"
            try:
                img = self.bridge.imgmsg_to_cv2(msg,msg.encoding)
            except CvBridgeError as e:
                rospy.logerr('image_cb failed: {}'.format(e))

            for i in range(len(self.u_v_list)):
                if (i < len(self.u_v_list)/2):
                    color=(0,0,255)
                else:
                    color=(255,0,0)
                if (i == self.cursor or i == self.cursor + len(self.u_v_list)/2):
                    size=16
                else:
                    size=10
                img_circle = cv2.circle(img,(int(self.u_v_list[i][0]),int(self.u_v_list[i][1])),size,color,-1)
                img_msg = self.bridge.cv2_to_imgmsg(img,msg.encoding)
                self.pub_image.publish(img_msg)
        else:
            self.pub_image.publish(msg)
        # img_circle = cv2.circle(img,(int(self.u),int(self.v)),10,(255,0,0),-1) #point_stamped_cb

    #use service
    def transform(self, req):
        rospy.logdebug("in transform")
        if not self.is_camera_arrived:
            return
        try:
            # transform = self.tf_buffer.lookup_transform(self.frame_id, self.header_frame_id, rospy.Time(0), rospy.Duration(1.0))
            transform = self.tf_buffer.lookup_transform(self.frame_id, req.frame_id.data, rospy.Time(0), rospy.Duration(1.0)) #lleg_end_coords was noting,then use BODY frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup_transform failed: {}'.format(e))
            return
        self.flag=True
        u_v_list_tmp = []
        points_list=req.l_points + req.r_points
        for i in range (len(points_list)):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = points_list[i].x * 0.001
            pose_stamped.pose.position.y = points_list[i].y * 0.001
            pose_stamped.pose.position.z = points_list[i].z * 0.001
            
            position_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose.position
            pub_point = (position_transformed.x, position_transformed.y, position_transformed.z)
            u, v = self.cameramodels.project3dToPixel(pub_point)
            rospy.logdebug("u, v : {}, {}".format(u, v))
            u_v_list_tmp.append([u,v])
        self.u_v_list = u_v_list_tmp
        print ("u_v_list_tmp={}".format(u_v_list_tmp))
        
    def point_stamped_cb(self, msg):
        if not self.is_camera_arrived:
            return
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = msg.point.x
        pose_stamped.pose.position.y = msg.point.y
        pose_stamped.pose.position.z = msg.point.z
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup_transform failed: {}'.format(e))
            return
        position_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose.position
        pub_point = (position_transformed.x, position_transformed.y, position_transformed.z)
        self.u, self.v = self.cameramodels.project3dToPixel(pub_point)
        rospy.logdebug("u, v : {}, {}".format(self.u, self.v))
        
        # img1 = self.bridge.imgmsg_to_cv2(msg2,msg1.encoding)
        # pub_msg = PointStamped()
        # pub_msg.header = msg.header
        # pub_msg.header.frame_id = self.frame_id
        # pub_msg.point.x = u
        # pub_msg.point.y = v
        # pub_msg.point.z = 0
        # self.pub.publish(pub_msg)

    def polygon_stamped_cb(self, msg):
        if not self.is_camera_arrived:
            return
        
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup_transform failed: {}'.format(e))
            return
        u_v_list_tmp = []
        for i in range (len(msg.polygon.points)):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = msg.polygon.points[i].x
            pose_stamped.pose.position.y = msg.polygon.points[i].y
            pose_stamped.pose.position.z = msg.polygon.points[i].z
            
            position_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose.position
            pub_point = (position_transformed.x, position_transformed.y, position_transformed.z)
            u, v = self.cameramodels.project3dToPixel(pub_point)
            rospy.logdebug("u, v : {}, {}".format(u, v))
            u_v_list_tmp.append([u,v])
        self.u_v_list = u_v_list_tmp
        
if __name__ == '__main__':
    print("OK")
    rospy.init_node("PutPointsOnImage")
    PutPointsOnImage_obj = PutPointsOnImage()
    rospy.spin()

# information of Joy topic  from  vive
# Trigger, Trackpad X, Trackpad Y
# axes: [0.0, 0.0, 0.0]
# Trigger, Trackpad touched, Trackpad pressed, Menu, Gripper
# buttons: [0, 0, 0, 0, 0]

# execute
# roslaunch put_points_on_image.launch
# rostopic pub -r 5 vive_left sensor_msgs/Joy '{header: {stamp: 'now', frame_id: 'HEAD_LINK0'}, axes: [0, 0, 0], buttons:[1, 0, 0, 0,0]}'
# rostopic pub -r 5 vive_right sensor_msgs/Joy '{header: {stamp: 'now', frame_id: 'HEAD_LINK0'}, axes: [0, 0, 0], buttons:[1, 0, 0, 0,0]}'
