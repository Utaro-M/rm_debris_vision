#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "switch_input_cloud_client")
(ros::wait-for-service "switch_pcl")

(setq tri nil)
(defun switch_input_cloud(camera)
  (let ((req (instance rm_debris_vision::SwitchFlagRequest :init)))
    (if (equal camera "multisense")
        (setq tri t)
      (setq tri nil))
    (format t "flag =~A~%" tri)
    (send req :trigger :data tri)
    (setq res (ros::service-call "switch_pcl" req t))
    (format t "camera = ~A~%" camera))
  )

;; (switch_input_cloud "multisense")
