#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "change_clip_box_server")

(setq clip-point #f(0 0 0))
(setq coords (make-coords))

(defun send_box(req)
  (let ((m (send req :response)))
    (format t "box size x = ~A~%" (send req :box_size :x))
    (format t "box size y = ~A~%" (send req :box_size :y))
    (format t "box size z = ~A~%" (send req :box_size :z))
    (setq clip-point-req (float-vector (send req :clip_point_req :x) (send req :clip_point_req :y) (send req :clip_point_req :z)))
    (if (not (send req :use_auto_clip_point :data))  ;;#f(100 100 100)が送られてきたときは自動で計算したclip-pointを使う
        (setq clip-point clip-point-req))
    (ros::set-dynamic-reconfigure-param "plane_extraction/bounding_box_publisher" "x" :double (elt clip-point 0))
    (ros::set-dynamic-reconfigure-param "plane_extraction/bounding_box_publisher" "y" :double (elt clip-point 1))
    (ros::set-dynamic-reconfigure-param "plane_extraction/bounding_box_publisher" "z" :double (elt clip-point 2))
    (ros::set-dynamic-reconfigure-param "plane_extraction/bounding_box_publisher" "w" :double (send req :box_size :x))
    (ros::set-dynamic-reconfigure-param "plane_extraction/bounding_box_publisher" "d" :double (send req :box_size :y))
    (ros::set-dynamic-reconfigure-param "plane_extraction/bounding_box_publisher" "h" :double (send req :box_size :z))    
    (send m :clip_point :x (elt clip-point 0))
    (send m :clip_point :y (elt clip-point 1))
    (send m :clip_point :z (elt clip-point 2))
    (format t "clip box size         = (~A ,~A ,~A)~%" (send req :box_size :x) (send req :box_size :y) (send req :box_size :z))
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

(ros::advertise-service "clip_box_point" rm_debris_vision::Clip_Point #'send_box)

(do-until-key
 (ros::spin-once)
)


