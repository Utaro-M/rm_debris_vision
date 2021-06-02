#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "rarm_remove_cloud_indices_server")

(defun send_indices(req)
  (let ((m (send req :response)))
    (send m :indices indices)
    m
  ))

;; callback function
(defun call-back (msg)
  (setq indices msg))

(setq indices (instance pcl_msgs::PointIndices :init))
(ros::subscribe "plane_extraction/rarm_indices" pcl_msgs::PointIndices #'call-back)
(ros::advertise-service "rarm_remove_cloud_indices" rm_debris_vision::Indices #'send_indices)

(do-until-key
 (ros::spin-once)
 ;;(sys::gc)
)
