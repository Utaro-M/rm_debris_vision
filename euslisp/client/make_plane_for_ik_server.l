#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")

(ros::roseus "make_plane_for_ik_server")
(setq points (instance geometry_msgs::PolygonStamped :init))
(setq tmp-point-list (send points :polygon :points))
(setq com-save #f(0 0 0))
(setq com-pre #f(0 0 0))
(setq normal-vec-save #f(0 0 0))
(setq normal-vec-pre #f(0 0 0))
(defun send_plane(req)
  (let ((m (send req :response)))
    (print "send_plane server")
    (send m :pts points)
    (send (send m :normal_vec) :x (elt normal-vec-save 0)) (send (send m :normal_vec) :y (elt normal-vec-save 1)) (send (send m :normal_vec) :z (elt normal-vec-save 2))
    (send (send m :com) :x (elt com-save 0)) (send (send m :com) :y (elt com-save 1)) (send (send m :com ):z (elt com-save 2))
    m
  ))

(defun call-back (msg)
  (setq points msg)
  (setq point-list (send points :polygon :points))
  (setq vertices (mapcar #'(lambda (vertice) (scale 1000 (float-vector (send vertice :x) (send vertice :y) (send vertice :z)))) point-list))
  (setq my-plane (make-prism vertices 10))

  (setq com-tmp (send my-plane :centroid))
  (setq com-save (scale 0.5 (v+ com-tmp com-pre)))
  (setq com-pre com-tmp)

  (setq normal-vec (send (elt (send my-plane :get-face :prism :top) 0) :normal))
  (setq normal-vec-save (scale 0.5 (v+ normal-vec normal-vec-pre)))
  (setq normal-vec-pre normal-vec)

  ;; (format t "com-save =~A~%" com-save)
  ;; (format t "normal-vec-save =~A~%" normal-vec-save)  
  )

(ros::subscribe "get_plane/focus_plane" geometry_msgs::PolygonStamped #'call-back)

(ros::advertise-service "make_plane" rm_debris_vision::Polygon #'send_plane)
(setq cnt 0)

(do-until-key
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
  (ros::spin-once)
 )
