#!/usr/bin/env roseus

;; put_points_on_image.py
(ros::load-ros-manifest "rm_debris_vision")
(require "package://rm_debris_vision/euslisp/functions.l")
(require "package://rm_debris_vision/euslisp/client/make_plane_for_opt.l ")
(ros::roseus "display_grasp_candidates_server")
(ros::wait-for-service "display_grasp_candidates")

(setq debug t)
(setq candidates-coords-list (list)
      start-coords-list (list)
      goal-coords-list (list))

;; put_points_on_image.pyへcandidatesを送る hands-intervalは両手の間隔
(defun send-grasp-candidates(&key (hands-interval (list  100 300 500)))
  (setq req (instance rm_debris_vision::GraspCandidatesRequest :init))
  (let*
      ((lis (list))
       (points-l-list (list))
       (points-r-list (list))
       (coords-tmp )
       (l-pos-wrt #f(0 0 0))
       (r-pos-wrt #f(0 0 0))
       (l-pos #f(0 0 0))
       (r-pos #f(0 0 0))
       (frame_id (instance  std_msgs::String :init)))
    (setq candidates-coords-list (list))
    (dotimes (i (length hands-interval))
      (setq coords-tmp (auto-set-reaching-targets-each-hand :pos-shift (elt hands-interval i)))
      (setq candidates-coords-list (list-insert coords-tmp 100 candidates-coords-list))
      
      (setq waist-coords (send (car (send *robot* :links)) :copy-worldcoords))
      
      (setq l-pos-wrt (send (send (elt coords-tmp 0) :copy-worldcoords) :worldpos))      
      (setq l-pos (send waist-coords :inverse-transform-vector l-pos-wrt)) ;; transformed BODY frame
      (setq point-l (instance  geometry_msgs::Point32 :init)) ;; left
      (send point-l :x (elt l-pos 0))
      (send point-l :y (elt l-pos 1))
      (send point-l :z (elt l-pos 2))

      (setq r-pos-wrt (send (send (elt coords-tmp 1) :copy-worldcoords) :worldpos))      
      (setq r-pos (send waist-coords :inverse-transform-vector r-pos-wrt)) ;; transformed BODY frame      
      (setq point-r (instance  geometry_msgs::Point32 :init)) ;; right
      (send point-r :x (elt r-pos 0))
      (send point-r :y (elt r-pos 1))
      (send point-r :z (elt r-pos 2))
      
      (format t "point-l=~A~%" point-l)
      (setq points-l-list (list-insert point-l 100 points-l-list))
      (setq points-r-list (list-insert point-r 100 points-r-list))      
      )
    ;; request
    (send req :l_points points-l-list)
    (send req :r_points points-r-list)
    (send frame_id :data "BODY")
    (send req :frame_id frame_id)
    (format t "l-list = ~A~%" points-l-list)
    (format t "candidates-coords-list = ~A~%" candidates-coords-list)    
    (setq res (ros::service-call "display_grasp_candidates" req t))
    )
  )

(defun cb (req)
  (format t "in cb euslisp~%")
  (let ((m (send req :response)))
    (cond ((equal "get_candidates" (send req :function)) ;;  return cnadidates
           (progn
             (format t "get_candidates~%")
             (send-grasp-candidates :hands-interval (send req :data))))

          ((equal "reach_candidates" (send req :function))
           (progn ;;  solve ik
             (format t "reach candidates~%")
             (format t "req = ~A~%" (elt (send req :data) 0))
             (setq coords-list (list (elt (elt candidates-coords-list (elt (send req :data) 0)) 0) (elt (elt candidates-coords-list (elt (send req :data) 0)) 1)))
             (format t "coords-list = ~A~% " coords-list)
             (send *robot* :angle-vector (send *ri* :state :angle-vector))
             (reset-coords)
             ;; (ik-opt (elt coords-list 0) (elt coords-list 1) :only-kinematics nil  :optimize-torque? nil :fix-contact? 2 :avoid-collision? t :apply-external-force? nil :wait? t :rleg-pos-shift #f(0 0 0) :lleg-pos-shift #f(0 0 0)   :set-unique-kin-scale? t :obst? nil)
             ;; (send *robot-env* :dissoc-root-virtual)
             (if (ik (elt coords-list 0) (elt coords-list 1)) ;; general ik
                 (send m :res t) ;;ik success
               (progn
                 (format t "ik failed~%")
                 (send m :res nil)
                 (return-from cb m))) ;;ik fail
             (if (not debug)
                 (stop-master-slave))
             (send-pose-and-move-on-foot :time 8000 :send? t :fix-contact? 2   :look-at-object? nil)
             ))

          ((equal "set_startpos" (send req :function))
           (progn
             (format t "set start pos~%")
             (if (not debug)
                 (stop-master-slave))
             (setq start-coords-list (get-coords "slave")))
           )

          ((equal "set_goalpos" (send req :function))
           (progn
             (format t "set goal pos~%")
             (setq goal-coords-list (get-coords "slave"))
             (send *robot* :angle-vector (send *ri* :state :angle-vector))
             (setq diff-list (list))
             (dotimes (i (length start-coords-list))
               (progn
                 (setq diff-list (append (list (send (elt start-coords-list i) :transformation (elt goal-coords-list i))) diff-list))
                 ))
             (reset-coords)
             (ik (send (send (send *robot* :rarm :end-coords) :copy-worldcoords)  :transform (elt diff-list 0))
                 (send (send (send *robot* :larm :end-coords) :copy-worldcoords)  :transform (elt diff-list 1))) ;; general ik
             (if (not debug)
                 (stop-master-slave))
             (send-pose-and-move-on-foot :time 8000 :send? t :fix-contact? 2   :look-at-object? nil))
           )
          )
    (format t "receive trigger_with_data")
    m)
  )
;;rh lh rf lf
(defun get-coords(slave-or-master)
  (let* ((msg-list (list (one-shot-subscribe  (format nil "/~A_rarm_pose" slave-or-master) geometry_msgs::PoseStamped :timeout 3000)
                        (one-shot-subscribe  (format nil "/~A_larm_pose" slave-or-master) geometry_msgs::PoseStamped :timeout 3000)
                        (one-shot-subscribe  (format nil "/~A_rleg_pose" slave-or-master) geometry_msgs::PoseStamped :timeout 3000)
                        (one-shot-subscribe  (format nil "/~A_lleg_pose" slave-or-master) geometry_msgs::PoseStamped :timeout 3000))))
             (setq coords-list
                   (mapcar #'(lambda (msg)
                               (make-coords :pos (funcall #'(lambda (pos) (scale 1000 (float-vector (send pos :x) (send pos :y) (send pos :z)))) (send msg :pose :position))
                                            :rot (funcall #'(lambda (ori) (quaternion2matrix (float-vector (send ori :x) (send ori :y) (send ori :z) (send ori :w)))) (send msg :pose :orientation))))
                           msg-list))
             (format t "get ~A coords = ~A" slave-or-master coords-list)
             coords-list
             )
  )

;; general ik
(defun ik(larm rarm)
  (setq flag
        (send *robot* :fullbody-inverse-kinematics
              (list (send *robot* :rleg :end-coords :copy-worldcoords)
                    (send *robot* :lleg :end-coords :copy-worldcoords)
                    rarm
                    larm)
              :move-target
              (list (send *robot* :rleg :end-coords)
                    (send *robot* :lleg :end-coords)
                    (send *robot* :rarm :end-coords)
                    (send *robot* :larm :end-coords))
              :link-list
              (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
                    (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
                    (send *robot* :link-list (send *robot* :rarm :end-coords :parent))
                    (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
              :translation-axis (list t t t t)
              :rotation-axis (list t t :z :z)
              ;; :dump-command :fail-only
              :warnp nil
              :target-centroid-pos (midpoint 0.5
                                             (send *robot* :rleg :end-coords :worldpos)
                                             (send *robot* :lleg :end-coords :worldpos))
              :cog-translation-axis :z))
        ;; (send *obj* :locate (send *robot* :centroid) :world)
  (send *irtviewer* :draw-objects)
  flag)

(defun stop-master-slave()
  (send *ri* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :state :angle-vector)
  (send *ri* :wholebodymasterslaveservice_stopwholebodymasterslave)
  )

(defun start-master-slave()
  (send *ri* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :state :angle-vector)
  (send *ri* :wholebodymasterslaveservice_startwholebodymasterslave)
  )

; (ros::advertise-service "joy_trigger" rm_debris_vision::TriggerWithData #'cb)
(ros::advertise-service "joy_trigger" rm_debris_vision::TriggerWithString #'cb)

(do-until-key
  (ros::spin-once)
 )
