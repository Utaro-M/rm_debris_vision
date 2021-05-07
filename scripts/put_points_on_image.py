#!/usr/bin/env python

import rospy
import numpy as np
# import message_filters
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon #test
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rm_debris_vision.srv import GraspCandidates 

class PutPointsOnImage(object):
    def __init__(self):
        
        self.cameramodels = PinholeCameraModel()
        self.is_camera_arrived = False
        self.frame_id = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        self.flag=False
        self.u = 0
        self.v = 0
        self.u_v_list = []
        self.header_frame_id=""

        # publish
        self.pub_point_stamped = rospy.Publisher("~output", PointStamped, queue_size=1)
        self.pub_image = rospy.Publisher("~output/image", Image, queue_size=1)

        # subscribe
        rospy.Subscriber('~input/camera_info', CameraInfo, self.camera_info_cb)
        rospy.Subscriber("~input/image", Image, self.image_cb)
        self.s=rospy.Service("display_grasp_candidates", GraspCandidates,self.display) #use service
        # rospy.Subscriber("~input/polygon_stamped", PolygonStamped, self.polygon_stamped_cb) #subscribe points
        # rospy.Subscriber("~input/point_stamped", PointStamped, self.point_stamped_cb)
                
    def camera_info_cb(self, msg):
        self.cameramodels.fromCameraInfo(msg)
        self.frame_id = msg.header.frame_id
        self.is_camera_arrived = True

    # add circles
    def image_cb(self,msg):
        self.header_frame_id = msg.header.frame_id
        if self.flag :
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
                img_circle = cv2.circle(img,(int(self.u_v_list[i][0]),int(self.u_v_list[i][1])),10,color,-1)
                img_msg = self.bridge.cv2_to_imgmsg(img,msg.encoding)
                self.pub_image.publish(img_msg)
        else:
            self.pub_image.publish(msg)
        # img_circle = cv2.circle(img,(int(self.u),int(self.v)),10,(255,0,0),-1) #point_stamped_cb

    #use service
    def display(self, req):
        rospy.logdebug("in display")

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
