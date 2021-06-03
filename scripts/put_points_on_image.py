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
from rm_debris_vision.srv import TriggerWithString

use="eus"

class PutPointsOnImage(object):
    def __init__(self):
        
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False
        self.frame_id = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.state = "sleep"
        self.u_v_list = []
        self.header_frame_id=""
        self.hands_interval=[100,300,500]
        self.cursor=0
        self.count_threshould=5
        self.left_flag_count =[0, 0, 0, 0, 0]
        self.right_flag_count =[0, 0, 0, 0, 0]
        # publish
        # self.pub_point_stamped = rospy.Publisher("~output", PointStamped, queue_size=1)
        # self.pub_image = rospy.Publisher("~output/image", Image, queue_size=1)

        # subscribe
        # rospy.Subscriber('~input/camera_info', CameraInfo, self.camera_info_cb)
        # rospy.Subscriber("~input/image", Image, self.image_cb)
        self.left_transformer  = DisplayTransformedPoints("~input/image_left",'~input/camera_info_left',"~output/image_left")
        self.right_transformer = DisplayTransformedPoints("~input/image_right",'~input/camera_info_right',"~output/image_right")
        #Joy
        rospy.Subscriber('~vive_left', Joy, self.joy_cb_left)
        rospy.Subscriber('~vive_right', Joy, self.joy_cb_right)

        if use =="eus":
            self.s=rospy.Service("display_grasp_candidates", GraspCandidates,self.transform) #use service
        elif use =="pca":
            rospy.Subscriber("~input/pose_array", PoseArray, self.pca_cb) #subscribe points
        # rospy.Subscriber("~input/polygon_stamped", PolygonStamped, self.polygon_stamped_cb) #subscribe points
        # rospy.Subscriber("~input/point_stamped", PointStamped, self.point_stamped_cb)
        rospy.wait_for_service('joy_trigger')
        
    # def camera_info_cb(self, msg):
    #     self.cameramodels.fromCameraInfo(msg)
    #     self.frame_id = msg.header.frame_id
    #     self.is_camera_arrived = True

    # detect button and change state
    # state :: sleep -> button input (change mode)-> choosing (wait for operator instruction) -> button input (operator chooses target candidates) -> sleep (advertise choosed candidates and request ik solving)
    def joy_cb_left(self,msg):
        self.set_button_count(msg,"left")
        print("left count : {}".format(self.left_flag_count))
        self.change_state()

    def joy_cb_right(self,msg):
        self.set_button_count(msg,"right")
        print("right count : {}".format(self.right_flag_count))
        self.change_state()

    def set_state(self,state):
        self.state = state
        self.left_transformer.state = state
        self.right_transformer.state = state
        print("\nset state -> {}\n".format(state))

    def set_cursor(self):
        if self.pushed("left",2):  # :: change candidates
            self.cursor +=1
        elif self.pushed("right",2): #:: change candidates
            self.cursor -=1
        if(self.cursor < 0):
            self.cursor = 0
        if(len(self.u_v_list)/2 <= self.cursor):
            self.cursor = len(self.left_transformer.u_v_list)/2 -1
        self.left_transformer.cursor = self.cursor
        self.right_transformer.cursor = self.cursor
        print("\nset cursor -> {}\n".format(self.cursor))        
        
    def change_state(self):
        if self.pushed("left",0): #reset
            self.set_state("sleep")
            self.reset_button_count("both")
            
        if (self.state == "sleep"):
            if self.pushed("left",3) and self.pushed("right",3): #  state: sleep -> choosing
                self.send_trigger("get_candidates",self.hands_interval)
                self.set_state("choosing")
            if self.pushed("left",2) and self.pushed("right",2):
                self.send_trigger("set_startpos",None) #first: set start pos
                self.set_state("auto_move")

        elif(self.state == "choosing"):
            self.set_cursor()
            if self.pushed("left",3) and self.pushed("right",3): #  state: choosing -> sleep
                # self.send_trigger(False,[self.cursor])
                self.send_trigger("reach_candidates",[self.cursor])
                self.set_state("sleep")
                
        elif(self.state == "auto_move"):
            if self.pushed("left",2) and self.pushed("right",2):
                self.send_trigger("set_goalpos",None) #second: set goal pos
                self.set_state("sleep")
                
        print("state = {}".format(self.state))
        # rospy.wait_for_service('auto_ik_trigger')

    # when state==choosing : add circles to image and publish it
    # when state == otherwise : only publish original image
        
    def set_button_state(self,left_msg,right_msg):
        print("in set_button_state : {}".format(self.left_flag_count))
        for i in range (len(left_msg.buttons)):
            if left_msg.buttons[i]==1:
                self.left_flag_count[i] += 1
            else:
                self.left_flag_count[i] = 0
                
            if right_msg.buttons[i]==1:
                self.right_flag_count[i] += 1
            else:
                self.right_flag_count[i] = 0

    def set_button_count(self,msg,hand):
        for i in range (len(msg.buttons)):
            if msg.buttons[i]==1:
                if hand == "left":
                    self.left_flag_count[i] += 1
                else:
                    self.right_flag_count[i] += 1
            else:
                if hand == "left":
                    self.left_flag_count[i] = 0
                else:
                    self.right_flag_count[i] = 0

    def reset_button_count(self,hand):
        for i in range (5):
            if hand == "left":
                self.left_flag_count[i] = 0
            elif hand== "right":
                self.right_flag_count[i] = 0
            else:
                self.left_flag_count[i] = 0
                self.right_flag_count[i] = 0
        print("reset {} hand count".format(hand))
                
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
    def send_trigger(self,function,data):
        # rospy.wait_for_service(svc_name)
        try:
            Trigger_with_string = rospy.ServiceProxy('joy_trigger', TriggerWithString)
            resp = Trigger_with_string(function,data)
            print("\n joy_trigger res = {}".format(resp.res))
            if resp.res == False:
                self.set_state("sleep")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        self.reset_button_count("both")       

    #use service
    def transform(self, req):
        rospy.logdebug("in transform")
        self.left_transformer.transform(req)
        self.right_transformer.transform(req)
        # if not self.is_camera_arrived:
        #     return
        # try:
        #     # transform = self.tf_buffer.lookup_transform(self.frame_id, self.header_frame_id, rospy.Time(0), rospy.Duration(1.0))
        #     transform = self.tf_buffer.lookup_transform(self.frame_id, req.frame_id.data, rospy.Time(0), rospy.Duration(1.0)) #lleg_end_coords was noting,then use BODY frame
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logerr('lookup_transform failed: {}'.format(e))
        #     return
        
        # u_v_list_tmp = []
        # points_list=req.l_points + req.r_points
        # for i in range (len(points_list)):
        #     pose_stamped = PoseStamped()
        #     pose_stamped.pose.position.x = points_list[i].x * 0.001
        #     pose_stamped.pose.position.y = points_list[i].y * 0.001
        #     pose_stamped.pose.position.z = points_list[i].z * 0.001
            
        #     position_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose.position
        #     pub_point = (position_transformed.x, position_transformed.y, position_transformed.z)
        #     u, v = self.cameramodels.project3dToPixel(pub_point)
        #     rospy.logdebug("u, v : {}, {}".format(u, v))
        #     u_v_list_tmp.append([u,v])
        # self.u_v_list = u_v_list_tmp
        # print ("u_v_list_tmp={}".format(u_v_list_tmp))

    def transform_pca(self):
        rospy.logdebug("in transform pca")
        if not self.is_camera_arrived:
            return
        try:
            # transform = self.tf_buffer.lookup_transform(self.frame_id, self.header_frame_id, rospy.Time(0), rospy.Duration(1.0))
            transform = self.tf_buffer.lookup_transform(self.frame_id, req.frame_id.data, rospy.Time(0), rospy.Duration(1.0)) #lleg_end_coords was noting,then use BODY frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup_transform failed: {}'.format(e))
            return
        u_v_list_tmp = []
        points_list=l_points + req.r_points
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
        
    def pca_cb(self, msg):
        points_list = []
        for poses in msg.poses:
            points_list.append([poses[i].x, poses[i].y, poses[i].z])

class DisplayTransformedPoints(object):
    def __init__(self,image_topic_in,camera_info_topic_in,image_topic_out):
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False
        self.frame_id = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.header_frame_id=""
        self.u_v_list = []
        self.state = "sleep"
        self.cursor=0
        
        # self.pub_point_stamped = rospy.Publisher("~output/point_stamped", PointStamped, queue_size=1)
        self.pub_image = rospy.Publisher(image_topic_out, Image, queue_size=1)
        rospy.Subscriber(camera_info_topic_in, CameraInfo, self.camera_info_cb)
        rospy.Subscriber(image_topic_in, Image, self.image_cb)

    def camera_info_cb(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.frame_id = msg.header.frame_id
        self.is_camera_arrived = True
        
    def image_cb(self,msg):
        self.header_frame_id = msg.header.frame_id
        if self.state == "choosing" :
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

    def transform(self, req):
        rospy.logdebug("in transform")
        if not self.is_camera_arrived:
            return
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, req.frame_id.data, rospy.Time(0), rospy.Duration(1.0)) #lleg_end_coords was noting,then use BODY frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr('lookup_transform failed: {}'.format(e))
            return
        
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
