#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "change_hand_covered_check_area_server")

(setq clip-point #f(0 0 0))
(setq coords (make-coords))

(defun change-check-box(req)
  (let ((m (send req :response)))
    (format t "box size x = ~A~%" (send req :box_size_l :x))
    (format t "box size y = ~A~%" (send req :box_size_l :y))
    (format t "box size z = ~A~%" (send req :box_size_l :z))
    
    (setq clip-point-r (float-vector (send req :clip_point_req_r :x) (send req :clip_point_req_r :y) (send req :clip_point_req_r :z)))
    ;; (if (not (send req :use_auto_clip_point :data))  ;;#f(100 100 100)が送られてきたときは自動で計算したclip-pointを使う
    (setq clip-point clip-point-r)

    (if (send req :target :data)
        (setq target "_cover_")
      (setq target "_"))
      
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "x" :double (elt clip-point 0))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "y" :double (elt clip-point 1))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "z" :double (elt clip-point 2))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "w" :double (send req :box_size_r :x))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "d" :double (send req :box_size_r :y))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "h" :double (send req :box_size_r :z))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "roll" :double (send req :box_tilt_r :z))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "pitch" :double (send req :box_tilt_r :y))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Ararm" target)) "yaw" :double (send req :box_tilt_r :x))

    (setq clip-point-l (float-vector (send req :clip_point_req_l :x) (send req :clip_point_req_l :y) (send req :clip_point_req_l :z)))
    ;; (if (not (send req :use_auto_clip_point :data))  ;;#f(100 100 100)が送られてきたときは自動で計算したclip-pointを使う
    (setq clip-point clip-point-l)    
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "x" :double (elt clip-point 0))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "y" :double (elt clip-point 1))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "z" :double (elt clip-point 2))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "w" :double (send req :box_size_l :x))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "d" :double (send req :box_size_l :y))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "h" :double (send req :box_size_l :z))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "roll" :double (send req :box_tilt_l :z))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "pitch" :double (send req :box_tilt_l :y))
    (ros::set-dynamic-reconfigure-param (read-from-string (format nil "in_hand_point/bounding_box_publisher~Alarm" target)) "yaw" :double (send req :box_tilt_l :x))
    
    (send m :clip_point_l :x (elt clip-point 0))
    (send m :clip_point_l :y (elt clip-point 1))
    (send m :clip_point_l :z (elt clip-point 2))
    (format t "target                = ~A~%" (read-from-string (format nil "in_hand_point/bounding_box_publisher~A" target)))
    (format t "clip box size         = (~A ,~A ,~A)~%" (send req :box_size_l :x) (send req :box_size_l :y) (send req :box_size_l :z))
    (format t "clip box tilt         = (~A ,~A ,~A)~%" (send req :box_tilt_l :x) (send req :box_tilt_l :y) (send req :box_tilt_l :z))
    (format t "clip box center point = (~A ,~A ,~A)~%" (elt clip-point 0) (elt clip-point 1) (elt clip-point 2))    
    m
    ))

(ros::advertise-service "change_clip_area" rm_debris_vision::Clip_Point_CoverCheck #'change-check-box)


(do-until-key
 (ros::spin-once)
)


