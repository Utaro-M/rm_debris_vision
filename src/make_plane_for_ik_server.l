#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/005_add_two_ints
;;;
(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "make_plane_for_ik_server")
(setq points (instance geometry_msgs::PolygonStamped :init))
(setq com-save #f(0 0 0)) 
(defun send_plane(req)
  (let ((m (send req :response)))
    (send m :pts points)
    m
  ))

;; callback function
(defun call-back (msg)
  (setq points msg)
  ;; (setq vertices (mapcar #'(lambda (vertice) (scale 1000 (float-vector (send vertice :x) (send vertice :y) (send vertice :z)))) (send msg :polygon :points)))
  ;; (setq len (length vertices))
  ;; (setq sum_x 0)
  ;; (setq sum_y 0)
  ;; (setq sum_z 0)
  ;; (setq ave 0)
  ;; (dotimes (i len)
  ;;     (setq sum_x (+ sum_x (elt (elt vertices i) 0)))
  ;;     (setq sum_y (+ sum_y (elt (elt vertices i) 1)))
  ;;     (setq sum_z (+ sum_z (elt (elt vertices i) 2)))      
  ;;     )
  
  ;; (setq ave  (float-vector (/ sum_x (float len)) (/ sum_y (float len)) (/ sum_z (float len))))
  ;; (setq com-save ave)
  ;; (setq tmp ave)
  ;; (format t "com = ~A~%" (scale 0.001 ave))
  ;; (send com :coords (make-coords :pos (scale 1000 tmp)))
  ;; (send com :coords (make-coords))
  ;; (send com :translate  tmp)
  ;; (setq face (make-prism vertices 10))
  ;; (objects (list face com coords))
  ;; (send com :translate (scale -1 tmp))
  )
;; (setq coords (make-coords))
;; (setq com (make-cube 100 100 100))
;; (send com :set-color #f(1 0 0))
(ros::subscribe "get_plane/focus_plane" geometry_msgs::PolygonStamped #'call-back)

(ros::advertise-service "make_plane" rm_debris_vision::Polygon #'send_plane)

(do-until-key
 (ros::spin-once)
 ;;(sys::gc)
)
