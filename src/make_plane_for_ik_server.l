#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "make_plane_for_ik_server")
(setq points (instance geometry_msgs::PolygonStamped :init))
(setq tmp-point-list (send points :polygon :points))
(setq com-save #f(0 0 0)) 
(defun send_plane(req)
  (let ((m (send req :response)))
    (send m :pts points)
    m
  ))

(defun call-back (msg)
  (setq points msg)
  )

(ros::subscribe "get_plane/focus_plane" geometry_msgs::PolygonStamped #'call-back)

(ros::advertise-service "make_plane" rm_debris_vision::Polygon #'send_plane)
(setq cnt 0)

(do-until-key
 (ros::spin-once)
 (setq cnt (+ cnt 1))
 ;;平面がない時はリセット
 (if (= cnt 10000000)
     (progn
        (if (equal tmp-point-list (send points :polygon :points))
            (progn
              (print "reset")
              (setq points (instance geometry_msgs::PolygonStamped :init))
              ))
        (setq tmp-point-list (send points :polygon :points))
        (setq cnt 0)))
 )
