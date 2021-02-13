#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "change_hand_covered_check_area_server")

(setq clip-point #f(0 0 0))
(setq coords (make-coords))

(defun send_box(req)
  (let ((m (send req :response)))
    (format t "box size x = ~A~%" (send req :box_size_l :x))
    (format t "box size y = ~A~%" (send req :box_size_l :y))
    (format t "box size z = ~A~%" (send req :box_size_l :z))
    
    (setq clip-point-r (float-vector (send req :clip_point_req_r :x) (send req :clip_point_req_r :y) (send req :clip_point_req_r :z)))
    (if (not (send req :use_auto_clip_point :data))  ;;#f(100 100 100)が送られてきたときは自動で計算したclip-pointを使う
        (setq clip-point clip-point-r))    

    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "x" :double (elt clip-point 0))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "y" :double (elt clip-point 1))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "z" :double (elt clip-point 2))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "w" :double (send req :box_size_r :x))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "d" :double (send req :box_size_r :y))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "h" :double (send req :box_size_r :z))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "roll" :double (send req :box_tilt_r :z))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "pitch" :double (send req :box_tilt_r :y))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_rarm" "yaw" :double (send req :box_tilt_r :x))

    (setq clip-point-l (float-vector (send req :clip_point_req_l :x) (send req :clip_point_req_l :y) (send req :clip_point_req_l :z)))
    (if (not (send req :use_auto_clip_point :data))  ;;#f(100 100 100)が送られてきたときは自動で計算したclip-pointを使う
        (setq clip-point clip-point-l))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "x" :double (elt clip-point 0))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "y" :double (elt clip-point 1))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "z" :double (elt clip-point 2))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "w" :double (send req :box_size_l :x))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "d" :double (send req :box_size_l :y))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "h" :double (send req :box_size_l :z))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "roll" :double (send req :box_tilt_l :z))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "pitch" :double (send req :box_tilt_l :y))
    (ros::set-dynamic-reconfigure-param "in_hand_point/bounding_box_publisher_cover_larm" "yaw" :double (send req :box_tilt_l :x))
    
    (send m :clip_point_l :x (elt clip-point 0))
    (send m :clip_point_l :y (elt clip-point 1))
    (send m :clip_point_l :z (elt clip-point 2))
    (format t "clip box size         = (~A ,~A ,~A)~%" (send req :box_size_l :x) (send req :box_size_l :y) (send req :box_size_l :z))
    (format t "clip box tilt         = (~A ,~A ,~A)~%" (send req :box_tilt_l :x) (send req :box_tilt_l :y) (send req :box_tilt_l :z))
    (format t "clip box center point = (~A ,~A ,~A)~%" (elt clip-point 0) (elt clip-point 1) (elt clip-point 2))    
    m
  ))

;; callback function
(defun call-back (msg)
  (let* ((origin-vertices-list (mapcar #'(lambda (vertice) (scale 1000 (float-vector (send vertice :x) (send vertice :y) (send vertice :z)))) (send msg :polygon :points)))
         (origin-face (make-prism origin-vertices-list 10)))

    (setq com-origin-obj (send origin-face :centroid))
    
    (setq new-vertices-list (mapcar #'(lambda (l) (progn (if (< (elt l 0) (elt com-origin-obj 0)) l))) origin-vertices-list))
    (setq new-vertices (intersection new-vertices-list origin-vertices-list)) ;;andを取る
    
    (setq new-obj (make-prism new-vertices 10))
    (setq clip-point (scale 0.001 (send origin-face :centroid)))
    (objects (list coords new-obj))
    (format t "clip point = ~A~%" clip-point)
    ))

(ros::subscribe "get_plane/focus_plane" geometry_msgs::PolygonStamped #'call-back)

(ros::advertise-service "clip_covered_area" rm_debris_vision::Clip_Point_CoverCheck #'send_box)

(do-until-key
 (ros::spin-once)
)


