#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "change_clip_box_server")

(setq clip-point #f(0 0 0))

(defun send_box(req)
  (let ((m (send req :response)))
    ;; (ros::set-dynamic-reconfigure-param "bounding_box_array_publisher" "boxes" :int 10)
    (format t "box size x = ~A~%" (send req :box_size :x))
    (format t "box size y = ~A~%" (send req :box_size :y))
    (format t "box size z = ~A~%" (send req :box_size :z))
    (send req :box_size :y)
    (send req :box_size :z)
    (send m :clip_point :x (elt clip-point 0))
    (send m :clip_point :y (elt clip-point 1))
    (send m :clip_point :z (elt clip-point 2))
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
    (setq clip-point (send origin-face :centroid))
    ;; (objects (list new-obj))
    (format t "clip point = ~A~%" clip-point)
    ))

(ros::subscribe "get_plane/focus_plane" geometry_msgs::PolygonStamped #'call-back)

(ros::advertise-service "clip_box_point" rm_debris_vision::Clip_Point #'send_box)

(do-until-key
 (ros::spin-once)
)


