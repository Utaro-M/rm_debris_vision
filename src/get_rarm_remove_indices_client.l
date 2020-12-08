#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "rarm_remove_cloud_indices_client")
(ros::wait-for-service "rarm_remove_cloud_indices")

(setq req (instance rm_debris_vision::IndicesRequest :init))
(setq res 0)
(setq res (ros::service-call "rarm_remove_cloud_indices" req t))
(print res)


;;;
(ros::advertise "rarm_remove_indices" pcl_msgs::PointIndices 1)
(ros::rate 100)
(while (ros::ok)
  (setq msg res)
  ;; (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "rarm_remove_indices" msg)
  (ros::sleep)
  )
(ros::roseus "shutdown")
(exit)
