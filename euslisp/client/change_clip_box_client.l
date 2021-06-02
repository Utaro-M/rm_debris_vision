#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "change_clip_box_client")
(ros::wait-for-service "clip_box_point")

(defun send_box_size (&key (box-position #f(100 100 100)) (size-vector #f(1 1 1)))

  (setq req (instance rm_debris_vision::Clip_PointRequest :init))
  (if (v= box-position #f(100 100 100))
      (send req :use_auto_clip_point :data t)
    (send req :use_auto_clip_point :data nil))

  (send req :box_size :x (elt size-vector 0))
  (send req :box_size :y (elt size-vector 1))
  (send req :box_size :z (elt size-vector 2))

  (send req :clip_point_req :x (elt box-position 0))
  (send req :clip_point_req :y (elt box-position 1))
  (send req :clip_point_req :z (elt box-position 2))
  
  (setq res (ros::service-call "clip_box_point" req t))
  (format t "clip box size         = (~A ,~A ,~A)~%" (elt size-vector 0) (elt size-vector 1) (elt size-vector 2))
  (format t "clip box center point = (~A ,~A ,~A)~%" (send res :clip_point :x) (send res :clip_point :y) (send res :clip_point :z))
)                                      
;; (send_box_size 1.0 0.1 1 1 1 1)
