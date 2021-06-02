(require "models/arrow-object.l")
;; (require "package://control_tools/log-tools.l")
(require "package://control_tools/play-hrpsysfile.l")
;; (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(load "./class.l")
(load "./my-ik.l")
(load "./stick-pose.l")
(load "./functions-without-ri.l")
;; (load "client/change_count_points_thre.l")
;; (load "client/change_clip_area_client.l" )
;; (load "./client/predict_k.l" )
;; (load "./client/make_plane_for_opt.l" )
;; (load "./old/predict_from_multi_pose.l")
;; (require "../../euslisp/save-log.l")
;;平面認識使わないときはcomment out
;; (load "./client/make_plane_for_opt.l")
;;(load "./client/switch_input_cloud_client.l")
;; (require "~/research/rm_debris/src/predict_from_multi_pose.l")


(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
 )
(setq movetime 16000)
;; (objects (list *robot*))
(unless (boundp 'flag-simulator)
  (setq flag-simulator nil) ;;simulatorではｔ
  (setq movetime 14000))

(setq *coords* (instance my-coords :init ))

(setq *ik-param* (instance my-ik-param :init))
(defun prepare-simulator()
  (setq flag-simulator t) ;;simulatorではｔ
  (setq movetime 4000)
  (send *ri* :start-auto-balancer :limbs (list :lleg :rleg :larm :rarm))
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 400 :k-r 200) ;;0116  
  (send *ri* :set-log-maxlength (* 500 90))
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 0.3 :frame "world")
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))
  ;; (setq wei 0)
  ;; (mapcar #'(lambda (vertice) (setq wei (+ wei (send  vertice :weight)))) (send *robot* :links))
  ;; (send *ik-param* :set-rleg-wrench-offset (float-vector 0 0 (* 0.01 (/ wei 2)) 0 0 0))
  ;; (send *ik-param* :set-lleg-wrench-offset (float-vector 0 0 (* 0.01 (/ wei 2)) 0 0 0))
  (format t "prepare-simulator~%")
  )

(defun prepare-real-machine(&key (set-ref-force-updater? t) (emergency-off? t))
  (send *ri* :start-auto-balancer :limbs (list :lleg :rleg :larm :rarm))
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 500 :k-r 200)
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 300 :k-r 200) ;;0110
  ;; (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 200 :k-r 200) ;;0110
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 400 :k-r 200) ;;0116
  (if set-ref-force-updater?
      (progn 
	(send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0 :frame "world")
	;; (send *ri* :set-reference-force-updater-param :larm :motion-dir (normalize-vector (send  (send *robot* :larm :end-coords :copy-worldcoords) :inverse-transform-vector #f(0 0 -1))) :p-gain 0.01 :update-time-ratio 1.0)
	;; (send *ri* :set-reference-force-updater-param :rarm :motion-dir (normalize-vector (send  (send *robot* :rarm :end-coords :copy-worldcoords) :inverse-transform-vector #f(0 0 -1))) :p-gain 0.01 :update-time-ratio 1.0)  
	(send *ri* :start-reference-force-updater :arms)
	(print "set ref force updater ON")))
  ;; (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  ;; (send *ri* :start-impedance :arms)
  (setq flag-simulator nil)
  (if emergency-off?
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st)))
  (send *ri* :set-log-maxlength (* 500 300))
  (setq movetime 12000)
  ;; (setq wei 0)
  ;; (mapcar #'(lambda (vertice) (setq wei (+ wei (send  vertice :weight)))) (send *robot* :links))
  ;; (send *ik-param* :set-rleg-wrench-offset (float-vector 0 0 (* 0.01 (/ wei 2)) 0 0 0))
  ;; (send *ik-param* :set-lleg-wrench-offset (float-vector 0 0 (* 0.01 (/ wei 2)) 0 0 0))
  (format t "prepare-real-machine~%")
  )

(defun send-pose
    (time &key (pre-pose (send *robot* :angle-vector)) (pre-lleg  (send *robot* :lleg :end-coords :copy-worldcoords) ) (force-move (send *ik-param* :get-force-move)) (wait? t) (resume? nil) (look-at-object? t))
  (if look-at-object?
      (progn
        ;; (send *robot* :head :look-at (v+ (midpoint 0.5 (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos) (send (send *robot* :larm :end-coords :copy-worldcoords) :worldpos)) #f(500 0 300)))
        ;; (if flag-simulator
        ;;     (setq look-dir #f(600 0 200))
        ;;   ;; (setq look-dir #f(600 0 600))
	;;   ;; (setq look-dir #f(700 0 300))
	;;   (setq look-dir #f(600 0 750))
	;; (send *robot* :head :look-at (v+ (midpoint 0.5 (send (send *robot* :rarm :end-coords :copy-worldcoords) :worldpos) (send (send *robot* :larm :end-coords :copy-worldcoords) :worldpos)) look-dir)) ;;l515
	;; (send *robot* :look-at-hand :arms)
	;; (if (not flag-simulator)
	;;     (send *robot* :head-neck-p :joint-angle (- (send *robot* :head-neck-p :joint-angle) 12))
	;;   )
	(print "look at object")
	;; (send *robot* :angle-vector (send *robot* :head :look-at (midpoint 0.5 (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(150 -640 0) :local) :worldpos) (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(150 640 0) :local) :worldpos))))
    (if flag-simulator
        ;; (setq look-dir #f(600 0 200))
      (send *robot* :head-neck-p :joint-angle 40)        
      (send *robot* :angle-vector (send *robot* :head :look-at (send (send (send (car (send *robot* :links)) :copy-worldcoords) :translate #f(1000 0 600)) :worldpos))))
        ))
  (objects)
  ;; (x::window-main-one)
  (if (or force-move (setq ans (y-or-n-p "Do you move real machine?")))
      (progn
        (send *ri* :angle-vector (send *robot* :angle-vector) time)
        
        (if wait?
            (progn
              (print "wait-interpolation")
              (send *ri* :wait-interpolation)))
        )
    (progn
      (if resume?
	  (progn
	    (send *robot* :angle-vector pre-pose)
	    (send *robot* :move-coords (send pre-lleg :translate #f(0 0 140)) (send *robot* :link "LLEG_LINK5"))
	    (objects)
	    ;; (x::window-main-one)
	    (print "pre pose"))
      ))
    ))

(defun move-on-foot
    (init-rcoords init-lcoords next-rcoords  next-lcoords &key (which? :lleg) (only-move? nil) (wait? t) (fix-hand? nil) (force-move nil))
  ;; (if (setq ans (y-or-n-p "move foot?"))
  (objects)
  (if (< 0 (- (elt (send next-lcoords :worldpos) 1) (elt (send init-lcoords :worldpos) 1)))
      (setq which? :lleg)
    (setq which? :rleg))
  (format t "init-lcoords = ~A~%" init-lcoords)
  (format t "init-rcoords = ~A~%" init-rcoords)
  (format t "next-lcoords = ~A~%" next-lcoords)
  (format t "next-rcoords = ~A~%" next-rcoords)
  (if (or force-move (setq ans (y-or-n-p "Do you move foot?")))
      (progn
        (if fix-hand?
            (progn
              (send *ri* :set-auto-balancer-param :is-hand-fix-mode t)
              (unix:sleep 1)))
        (if (equal which? :lleg)
            (if wait?
                (send *ri* :set-foot-steps
                      (list
                       (make-coords :coords init-rcoords :name :rleg)
                       (make-coords :coords  next-lcoords :name :lleg)
                       (make-coords :coords  next-rcoords :name :rleg)
                       ))
              (send *ri* :set-foot-steps-no-wait            
                    (list
                     (make-coords :coords init-rcoords :name :rleg)
                     (make-coords :coords  next-lcoords :name :lleg)
                     (make-coords :coords  next-rcoords :name :rleg)
                     
                     )))
          (if wait?
              (send *ri* :set-foot-steps
                    (list
                     (make-coords :coords init-lcoords :name :lleg)
                     (make-coords :coords  next-rcoords :name :rleg)
                     (make-coords :coords  next-lcoords :name :lleg)
                     ))
            (send *ri* :set-foot-steps-no-wait
                  (list
                   (make-coords :coords init-lcoords :name :lleg)
                   (make-coords :coords  next-rcoords :name :rleg)
                   (make-coords :coords  next-lcoords :name :lleg)
                   
                   ))
            ))
        ))
  (if only-move?
      (send *robot* :angle-vector (send *ri* :state :angle-vector)))
  (send *robot* :move-coords (send (send next-lcoords :copy-worldcoords) :translate #f(0 0 100)) (send *robot* :link "LLEG_LINK5"))
  (send *robot* :move-coords (send (send next-rcoords :copy-worldcoords) :translate #f(0 0 100)) (send *robot* :link "RLEG_LINK5"))
  ;; (send *robot* :move-coords (send (send next-lcoords :copy-worldcoords) :translate #f(0 0 40)) (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (send *robot* :move-coords (send (send next-rcoords :copy-worldcoords) :translate #f(0 0 40)) (send *robot* :lleg :end-coords :copy-worldcoords))
  (objects)
  ;; (send *ri* :angle-vector (send *ri* :potentio-vector))
  ;; (send *robot* :angle-vector (send *ri* :potentio-vector))
  (if fix-hand?
      (progn
        (send *ri* :set-auto-balancer-param :is-hand-fix-mode nil)
        (unix:sleep 1)))
  )

(defun send-pose-and-move-on-foot
    (&key  (fix-contact? t) (send? t) (only-move-foot? nil) (use-zmp-weight-map? t)
           (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
           (look-at-object? t)
           ;; (rleg-coords (send *coords*rds* :get-rleg-coords))
           ;; (lleg-coords (send *coords* :get-lleg-coords))
           (rleg-coords-new (send *coords* :get-rleg-new-coords))
           (lleg-coords-new (send *coords* :get-lleg-new-coords))
           (fix-hand? nil)
           (time movetime))
  (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq rleg-coords (send *coords* :get-rleg-coords))
  (setq lleg-coords (send *coords* :get-lleg-coords))
  (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  (send *coords* :set-latest-angle-vector (send *robot* :angle-vector))
  (if send?
      (progn 
        (if (not fix-contact?)
            (progn ;;踏み出しあり
              (if use-zmp-weight-map?
                  (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1)))
              (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? wait? :fix-hand? fix-hand?) ;;wait? t なら踏み出し後、姿勢変更　wait? nil なら姿勢変更同時 踏み出しの分だけ手を引く
              (if (not only-move-foot?)
                  (progn ;;姿勢も変更
                    (if (not wait?)
                        (progn ;;姿勢変更同時 踏み出しの分だけ手を引く
                          ;; (unix:usleep (round (* 400000 (send (send *ri* :get-gait-generator-param) :default_step_time))))
                          (unix:usleep (round (* 7000000 (send (send *ri* :get-gait-generator-param) :default_step_time))))
                          (print "sleep")
                          (setq movetime-tmp (* 2000 (send (send *ri* :get-gait-generator-param) :default_step_time)))
                          (setq force-move-tmp t)
                          (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
                          (setq diff (v- (midpoint 0.5 (send rleg-coords-new :worldpos) (send lleg-coords-new :worldpos)) (midpoint 0.5 (send rleg-coords :worldpos) (send lleg-coords :worldpos))))
                          (move-hand :rarm (scale -1 diff) :send? nil :frame :world)
                          (move-hand :larm (scale -1 diff) :send? nil :frame :world)
                          (format t "pull hands ~A~% "(scale -1 diff))
                          (send-pose movetime-tmp :force-move force-move-tmp :look-at-object? look-at-object? :wait? t)
                          (send *robot* :angle-vector (send *coords* :get-latest-angle-vector))
                          )
                      (progn ;;踏み出し後、姿勢変更
                        (setq movetime-tmp 5000)
                        (setq force-move-tmp flag-simulator)))
                    (send-pose time :force-move force-move-tmp :look-at-object? look-at-object? :wait? t)
                    )
                (format t "move only foot~%")
                ))
          (progn
            (setq force-move-tmp flag-simulator) ;;踏み出しなし
            (send-pose time :force-move force-move-tmp :look-at-object? look-at-object? :wait? wait?)))
        ))
  )

(defun reach
    (&key (fix-contact? nil)
          (send? t)
          (wait? t)
          (avoid-collision? nil)
          (apply-external-force? nil)
          (external-force-list  (list
				 (float-vector 0 0 400 0 0 0)
				 (float-vector 0 0 400 0 0 0)
				 (float-vector 0 0 -80 0 0 0)
				 (float-vector 0 0 -80 0 0 0)))
          ;; (external-coords-list (list (send *robot-env* :robot :get :rarm-contact-coords)
          ;;                             (send *robot-env* :robot :get :larm-contact-coords)))
          (rarm-coords (make-coords :pos (float-vector 390 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90))))
          (larm-coords (make-coords :pos (float-vector 390  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90))))
          (rleg-pos-shift #f(0 0 -10))
          (lleg-pos-shift #f(0 0 -10))
          (look-at-object? nil)
          )
  (send *robot* :reset-manip-pose)
  (send *coords* :set-rleg (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg (send *robot* :lleg :end-coords :copy-worldcoords))

  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :external-wrench-list external-wrench-list ;; :external-coords-list external-coords-list
          )

  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
;;   (setq zmp-vector (send *ri*  :zmp-vector))
;;   (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
;;   (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
;;   (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
;;   (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
;;   (if send?
;;       (progn 
;;         (if (not fix-contact?)
;;             (progn
;;               (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
;;               (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))
;; ;  (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog #f(0 0 1000))

;;         ;;もどす
;;         (send-pose movetime :force-move flag-simulator :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?  )))
  ;; (my-ik #f(0 0 40) #f(0 0 40))
  )

;;持ち上げる
(defun heave
    (rarm-move larm-move
               &key  (fix-contact? t) (send? t) (cog #f(0 0 700))
               (deg-r-x 0) (deg-r-y 0) (deg-r-z 0) (deg-l-x 0) (deg-l-y 0) (deg-l-z 0)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 0))
               (lleg-pos-shift #f(0 0 0))
               (look-at-object? nil))

  (setq rarm-coords (send (send (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move) :rotate (deg2rad deg-r-x) :x) :rotate (deg2rad deg-r-y) :y) :rotate (deg2rad deg-r-z) :z))
    (setq larm-coords (send (send (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (deg2rad deg-l-x) :x) :rotate (deg2rad deg-l-y) :y) :rotate (deg2rad deg-l-z) :z))

  (setq rleg-coords (make-coords :pos #f(0 -100 0))
        lleg-coords (make-coords :pos #f(0  100 0)))

  (setq rleg-coords-old (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-old (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force? :avoid-collision? avoid-collision? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift)

  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords-old lleg-coords-old rleg-coords-new lleg-coords-new :wait? nil)
              
  ;;             ))
  ;;       (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?)
  ;;       ))
  )

(defun change-arm
    (offset arm &key (send? t) (sync? t))

  (if sync?
     (send *robot* :angle-vector (send *ri* :state :angle-vector))) 
  ;;腕を引く
  (send *robot* arm :move-end-pos (scale offset #f(-1 0 0)) :local )
  (if send?
      (progn
        (send-pose (- movetime 1500) :force-move flag-simulator )))

  ;; (send *ri* :start-grasp arm)
  (send *ri* :move-gripper arm 0.4 :effort 10 :wait t) 
  ;;腕を下へ
  (if (eq arm :rarm)
      (send *robot* arm :move-end-pos (scale offset #f(0 1.4 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(0 -1.4 0)) :local))
  (if send?
      (progn
        (send-pose (- movetime 1500) :force-move flag-simulator )))

  ;;腕を戻す    
  (if (eq arm :rarm)
      (progn
        (send *robot* :larm :move-end-pos (scale offset #f(0 0.2 0)) :local)
        (send *robot* arm :move-end-pos (scale offset #f(1.7 0 0)) :local))
    (progn
      (send *robot* :rarm :move-end-pos (scale offset #f(0 -0.2 0)) :local)
    (send *robot* arm :move-end-pos (scale offset #f(1.7 0 0)) :local)))
  (if send?
      (progn
        (send-pose (- movetime 1500) :force-move flag-simulator )))
  
  )

(defun move-hand
    (arm relative-pos &key (time 4000) (send? t) (sync? nil) (wait? t) (frame :local) (rotation-axis? t))
  (if sync?
      (send *robot* :angle-vector (send *ri* :state :angle-vector)))
  (send *robot* arm :move-end-pos relative-pos frame :rotation-axis rotation-axis?)
  (objects (list *robot*))
  (if send?
      (progn
        (send-pose time :force-move flag-simulator :wait? wait?)))
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  )

(defun dive
    (rarm-move larm-move
               &key
               rarm-coords
               larm-coords
               (fix-contact? t) (send? t) (cog #f(0 0 700))
               (deg-r-x 0) (deg-r-y 0) (deg-r-z 0) (deg-l-x 0) (deg-l-y 0) (deg-l-z 0)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 -10))
               (lleg-pos-shift #f(0 0 -10))
               (look-at-object? nil)
               (face (cube 800 600 20))
               )

  (if (not rarm-coords)
      (setq rarm-coords (send (send (send (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate rarm-move) :rotate (deg2rad deg-r-x) :x) :rotate (deg2rad deg-r-y) :y) :rotate (deg2rad deg-r-z) :z)))
  (if (not larm-coords)
      (setq larm-coords (send (send (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (deg2rad deg-l-x) :x) :rotate (deg2rad deg-l-y) :y) :rotate (deg2rad deg-l-z) :z)))

  (setq rleg-coords-old (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-old (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (ik-opt-with-larmcontact rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force? :avoid-collision? avoid-collision? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :face face)
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn
  ;;       ;; (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg)
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords-old lleg-coords-old rleg-coords-new lleg-coords-new :wait? nil))
  ;;             )
  ;;       (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?)
  ;;       ))
  )

(defun dive-larm-only
    ( larm-move
               &key
               larm-coords
               (fix-contact? t) (send? t) (cog #f(0 0 700))
               (deg-r-x 0) (deg-r-y 0) (deg-r-z 0) (deg-l-x 0) (deg-l-y 0) (deg-l-z 0)
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 -10))
               (lleg-pos-shift #f(0 0 -10))
               (look-at-object? nil))

  ;; needless??
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (if (not larm-coords)
      (setq larm-coords (send (send (send (send (send *robot* :larm :end-coords :copy-worldcoords) :translate larm-move) :rotate (deg2rad deg-l-x) :x) :rotate (deg2rad deg-l-y) :y) :rotate (deg2rad deg-l-z) :z)))

  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (setq pre-lleg (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (ik-opt-with-larmcontact-only larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :reset? reset? :apply-external-force? apply-external-force? :avoid-collision? avoid-collision? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift)
  (if send?
      (progn
        (if (not fix-contact?)
            (progn
              (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
              (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords)   (send *robot* :lleg :end-coords :copy-worldcoords) :wait? nil)))
        (send-pose movetime :force-move flag-simulator :wait? wait? :pre-pose pre-pose :pre-lleg pre-lleg :look-at-object? look-at-object?)
        ))
  )
(setq flag-pcl 0)
;;thre-list rarm larm
;;output flag   0 ::both arms holding , 1 ::larm drop , 2 ::rarm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-force-pcl
    (&key (checking-time 6) (sleep-time 20000) (working? t) (thre-list (list 10 10)) (fin 3)) 
  (setq initial (send *robot* :angle-vector)
        rarm-f-pre  (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil)
        larm-f-pre  (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil)
        angle-pre (send *ri* :state :angle-vector)
        holding-flag '(0 0))

  (if working?
      (format t "~%~%check while working~%")
    (format t "~%~%check while still~%"))
  
  (dotimes (i (/ (* checking-time 1000000) sleep-time))
    (change-hand-cover-boxes) ;;cover check bocの更新
    (if (equal (send (send *ri* :get-st-param) :emergency_check_mode) 1)
        (progn
          (format t "emergency mode~%")
          (send *ri* :stop-motion)
          (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
          ;;CP error時には手先の目標力を10kg up
          (send *ik-param* :set-larm-wrench   (v+ (send *ik-param* :get-larm-wrench) #f(0 0 -100 0 0 0)))
          (send *ik-param* :set-rarm-wrench   (v+ (send *ik-param* :get-rarm-wrench) #f(0 0 -100 0 0 0)))
          (setq half-wrench (scale 0.5 (v+ (send *ik-param* :get-rarm-wrench) (send *ik-param* :get-larm-wrench))))
          (send *ik-param* :set-rleg-wrench   (v+ (send *ik-param* :get-rleg-wrench-offset) half-wrench))
          (send *ik-param* :set-lleg-wrench   (v+ (send *ik-param* :get-lleg-wrench-offset) half-wrench))
          
          ;; (setf (elt (send *ik-param* :get-external-wrench-list) 2) (v+ (elt (send *ik-param* :get-external-wrench-list) 2) #f(0 0 -100 0 0 0)))
          ;; (setf (elt (send *ik-param* :get-external-wrench-list) 3) (v+ (elt (send *ik-param* :get-external-wrench-list) 3) #f(0 0 -100 0 0 0)))
          (setq ret (ik-opt (send *coords* :get-rarm-target) (send *coords* :get-larm-target) :fix-contact? 2 :apply-external-force? (send *coords* :set-apply-external-force?) :wait? wait? ))
          (format t "change pose~%")
          ;; (if (equal ret ":loop-finished")
          ;;     (progn
          ;;       (format t "ik failed~%")
          ;;       (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
          ;;       (send *robot* :move-coords (send *coords* :lleg-contact-coords) (send *robot* :lleg :end-coords)))
            (send-pose-and-move-on-foot :time movetime :send? t :fix-contact? fix-contact? :look-at-object? t :wait? t :fix-hand? nil))
      )

    ;;Force 
    (format t "~%~%")
    (setq rarm-f  (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
    (setq holding-flag-force (list 0 0))
    ;;絶対値バージョン
    ;; (setq rarm-f-dif (abs (elt (v- rarm-f rarm-f-pre) 2)))
    ;; (setq larm-f-dif (abs (elt (v- larm-f larm-f-pre) 2)))  ;;0122 comment in
    (setq rarm-f-dif (elt (v- rarm-f rarm-f-pre) 2))
    (setq larm-f-dif (elt (v- larm-f larm-f-pre) 2));;0213 comment in
    (setq drop (list nil nil))
    (format t "rarm-f-dif       = ~A~%" rarm-f-dif)
    (format t "larm-f-dif       = ~A~%" larm-f-dif)

    ;;rarm
    (if (< (* +1 (elt thre-list 0)) rarm-f-dif)
        (progn
          (print "rarm force dif too large")
          (setf (car holding-flag-force) 0 ))
      (progn
          (setf (car holding-flag-force) 1)))
    ;;larm
    (if (< (* +1 (elt thre-list 1)) larm-f-dif)
      (progn
        (print "larm force dif too large")
        (setf (cadr holding-flag-force) 0))
        (progn
          (setf (cadr holding-flag-force) 1)))

    ;;Point Cloud
    (setq holding-flag-pcl (check-pcl))
    (format t "flags force pcl = ~A ~A ~%" holding-flag-force holding-flag-pcl)

    ;;drop判定 '(? ?) ?<- 0 or 1
    ;;rarm
    (if (or (= (car holding-flag-pcl) 2) (= (car holding-flag-pcl) 3)) ;;coverもしくは視野外のときは力判定、それ以外は視覚判定
        (progn
          (print "rarm covered")
          (setf (car holding-flag) (car holding-flag-force)))
      (progn
        (print "rarm not covered")
          (setf (car holding-flag) (car holding-flag-pcl))))
    ;;larm
    (if (or (= (cadr holding-flag-pcl) 2) (= (cadr holding-flag-pcl) 3)) ;;coverもしくは視野外のときは力判定、それ以外は視覚判定
        (progn
          (print "larm covered")
          (setf (cadr holding-flag) (cadr holding-flag-force)))
      (progn
        (print "larm not covered")
        (setf (cadr holding-flag) (cadr holding-flag-pcl))))

    ;何方かの手が外れていたら関数を抜ける
    (if (not (equal holding-flag '(1 1)))
        (return-from check-force-pcl holding-flag))
    
    (if working?
        (progn
          ;;動作終了判定
          (setq tmp (send *ri* :reference-vector))
          (setq angle-vector-dif (abs (norm (v- initial tmp))))
          (format t "angle-vector-dif = ~A~%" angle-vector-dif)
          (if (< angle-vector-dif 0.5)
              (progn
                (print "finish")
                ;; (setq holding-flag (list 10 10))
                (return angle-vector-dif))
            )
          (setq rarm-f-pre rarm-f)
          (setq larm-f-pre larm-f)))
    
    (setq angle-pre (send *ri* :state :angle-vector))
    
    ;; (unix:usleep (- sleep-time 10000))
    )
  (format t "holding flag = ~A~%" holding-flag)
  holding-flag
  )
;; (setq drop_flag_larm (instance std_msgs::Bool :init))
;; (setq drop_flag_rarm (instance std_msgs::Bool :init))
;; (send drop_flag_larm :data nil)
;; (send drop_flag_rarm :data nil)
(setq drop_flag_rarm 0)
(setq drop_flag_larm 1)
(setq drop_flag_list nil)
;;output flag   0 ::both arms holding , 1 ::larm drop , 2 ::rarm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-pcl()
  (let*
      ((holding_flag_list (one-shot-subscribe "/drop_flag_list" std_msgs::Int32MultiArray :timeout 5000))
       (flag 0))
    ;; (format t "drop_flag_list =~A~%" drop_flag_list)
    (if holding_flag_list
        (progn
          ;; (print "get holding_flag_list")
          (setq holding_flag_rarm (elt (send holding_flag_list :data) 0))
          (setq holding_flag_larm (elt (send holding_flag_list :data) 1))
          (setq cover_flag_rarm (elt (send holding_flag_list :data) 2))
          (setq cover_flag_larm (elt (send holding_flag_list :data) 3))
          (setq invision_flag_rarm (elt (send holding_flag_list :data) 4))
          (setq invision_flag_larm (elt (send holding_flag_list :data) 5)))
      (print "use old holding_flag_list"))
    
    ;; (format t "get rarm holding flag = ~A~%" holding_flag_rarm )
    ;; (format t "get larm holding flag = ~A~%" holding_flag_larm )    
    ;; (format t "get rarm cover flag = ~A~%" cover_flag_rarm )
    ;; (format t "get larm cover flag = ~A~%" cover_flag_larm )

    (if (= invision_flag_rarm 1) ;;in vision
        (if (= cover_flag_rarm 1)
            (progn                   ;;covered
              (print "rarm covered")
              (setq flag-r 2))
          (if (= holding_flag_rarm 1) ;;not covered
            (progn
              (print "rarm holding")
              (setq flag-r 1))
            (progn
              (print "rarm drop")
              (setq flag-r 0))))          
      (progn                     ;;out of vision
        (print "rarm out of vision")
        (setq flag-r 3)))
      
    (if (= invision_flag_larm 1) ;;in vision
        (if (= cover_flag_larm 1)
            (progn;; covered
              (print "larm covered")
              (setq flag-l 2))
          (if (= holding_flag_larm 1) ;;not covered
              (progn
                (print "larm holding")
                (setq flag-l 1))
            (progn
              (print "larm drop")
              (setq flag-l 0))))          
      (progn                  ;;out of vision
        (print "larm out of vision")
        (setq flag-l 3)))
    
    ;; (if (and (= holding_flag_larm 0) (= holding_flag_rarm 0))
    ;;     (progn
    ;;       (print "both arms drop")
    ;;       (setq flag-l 0)
    ;; (setq flag-l 0)))
    ;; flag)
    )
    (list flag-r flag-l)
  )


;;もとの姿勢に戻すか、正常なハンドの位置に合わせて戻すか
(defun retry-while-working (drop-arm &key (offset 140) (same-pose nil) (cog #f(100 0 0)))
  ;;一度手を引っ込める
  (if (eq drop-arm :rarm)
      (progn
        (heave (scale offset #f(-1 0.5 0)) #f(0 0 0))
        )
    (heave #f(0 0 0) (scale offset #f(-1 0.5 0) )))
  (print "pull back once")
  (send-pose movetime :force-move flag-simulator)
  
  ;;ロボットモデルを新しく作りriの現在姿勢をとる
  (setq *rbt* (instance jaxon_red-robot :init))
  (send *rbt* :angle-vector (send *ri* :state :angle-vector))
  (send *rbt* :fix-leg-to-coords (make-coords))
  (objects (list *robot* *rbt*))
  
  (setq rleg-coords (make-coords :pos #f(0 -100 0))
        lleg-coords (make-coords :pos #f(0  100 0)))

  ;;動作終了時
  ;;もとの姿勢に戻るかどうか
  (if same-pose
      (progn
        (format t "retry to same pose~%")
        (setq rarm-coodrs (send *rbt* :rarm :end-coords :copy-worldcoords))
        (setq larm-coodrs (send *rbt* :larm :end-coords :copy-worldcoords))
        (my-ik rarm-coords larm-coords :rleg rleg-coords :lleg lleg-coords :cog cog))
    (progn
      (format t "return to other arm's pos~%")
      (if (eq drop-arm :rarm)
          (progn
            (setq hold-arm :larm)
            (setq drop-arm-offset #f(50 50 0)))
        (progn
          (setq hold-arm :rarm)
          (setq drop-arm-offset #f(50 -50 0))))
      (format t "regrasp ~A ~%" drop-arm)
      (setq hold-arm-coords (send *robot* hold-arm :end-coords :copy-worldcoords))
      (send hold-arm-coords :translate #f(50 0 0) :local)
      (format t "hold-arm-coords = ~A~%" hold-arm-coords)
      ;;現在姿勢を利用（実機では余り腕がたれないかもその時はriを使う必要はない
      (setq hold-arm-coords-tmp (send *rbt* hold-arm :end-coords :copy-worldcoords))
      (format t "hold-arm-coords-tmp = ~A~%" hold-arm-coords-tmp)
      ;;y成分の符号を反転
      (aset (send hold-arm-coords-tmp  :worldpos) 1 (* -1 (elt (send hold-arm-coords-tmp :worldpos) 1)))
      ;;rpyのx，z成分の符号反転
      (setq rpy-tmp (float-vector (* -1 (elt (car (send (send hold-arm-coords-tmp :worldcoords) :rpy-angle)) 0 )) (elt (car (send (send hold-arm-coords-tmp :worldcoords) :rpy-angle)) 1 ) (* -1 (elt (car (send (send hold-arm-coords-tmp :worldcoords) :rpy-angle)) 2 ))))

      (setq drop-arm-target-coords (make-coords :pos (send hold-arm-coords-tmp :worldpos)  :rpy rpy-tmp))
      (send drop-arm-target-coords :translate drop-arm-offset :local)
      (format t "drop-arm-target-coords = ~A~%" drop-arm-target-coords)

      
      (if (eq drop-arm :rarm)
          (progn
            (my-ik drop-arm-target-coords hold-arm-coords :rleg rleg-coords :lleg lleg-coords :cog cog))
            (progn
              (my-ik hold-arm-coords drop-arm-target-coords :rleg rleg-coords :lleg lleg-coords :cog cog)))
      ))
  (send-pose movetime :force-move flag-simulator)

  )

(defun retry-flag-checker
    (flag)
  (cond
   ((eq 1 flag)
    (retry-while-working :rarm ))
   ((eq 2 flag)
         (retry-while-working :larm ))
   (t
    (print "do nothing"))))

(defun start-coords()
  ;; (send *robot* :move-coords (make-coords :pos  #f(0 100 40)) (send *robot* :lleg :end-coords))
  (send *robot* :move-coords (make-coords :pos #f(0 100 100) :rpy #f(0 0 0) ) (send *robot* :link "LLEG_LINK5"))
  (objects))

  
;;手先位置は固定で腰を高くするheight 1000とか
;;stand-up 1000 :cog #f( 100 0 800)
(defun stand-up
    (height &key (cog #f(0 0 700)) (degree 0) (move-root-link :z))

  (setq rarm-coords-const (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq larm-coords-const (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rleg-coords  (send *robot* :rleg :end-coords :copy-worldcoords)
        lleg-coords  (send *robot* :lleg :end-coords :copy-worldcoords))

  ;; (send *robot* :fix-leg-to-coords (make-coords))
  ;; (send *robot* :reset-manip-pose)
  ;; (send *robot* :reset-pose)

  (setq waist-pos (send (car (send *robot* :links)) :worldpos))
  ;; (aset waist-pos 2 height)
  (setq coo (make-coords :pos (float-vector (elt waist-pos 0) (elt waist-pos 1) height) :rpy (float-vector 0 (deg2rad 40) 0)))
  (send *robot* :move-coords  coo (car (send *robot* :links)))
  ;; (send *robot* :move-coords (make-coords :pos waist-pos :rpy (float-vector 0 (deg2rad 30) 0)) (car (send *robot* :links)))
  (objects)
  ;; (do-until-key (x::window-main-one))
  
  (my-ik rarm-coords-const larm-coords-const :rleg rleg-coords :lleg lleg-coords :cog cog :move-root-link move-root-link :virtual-joint-weight #f(0.5 0.5 0.5 0 0 0))
  ;; (objects)(read-line)
  (send-pose movetime :force-move flag-simulator)
  )

(defun push-board ()
  (if auto-detect-p
      (send *ri* :set-object-turnaround-ref-force :max-ref-force (* 9.8 6.0))
    (progn
      (send *ri* :set-ref-force (float-vector 0 0 (* 9.8 2.5 -1)) 2000 :arms)
      (send *ri* :wait-interpolation-seq))))
  
(defun prepare-ex ()
  (unless (= (send (send *ri* :get-st-param) :emergency_check_mode) 0)
    (warn "disable emergency-check~%")
    ;;COPのemergencyをきる
    (send *ri* :stop-st)
    (send *ri* :set-st-param :emergency-check-mode 0)
    (send *ri* :start-st))
  )

(defun log (name ang)
  (send *ri* :start-log)
  (unix:sleep 3)
  (my-init-pose)
  (unix:sleep 2)
  (reach)
  (unix:sleep 2)
  
  (save-log :fname name)
  )

(defun my-init-pose
    (&key (move-foot? nil) (only-upper? nil) (send? t) (tilt? nil))
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (send (send *robot* :joint "RLEG_JOINT3") :min-angle 0)
  (send (send *robot* :joint "LLEG_JOINT3") :min-angle 0)
  (objects (list *robot*))
  (print "init pose ")
  ;; (send *robot* :fix-leg-to-coords (make-coords))
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))

  (if flag-simulator
      (send *robot* :head-neck-p :joint-angle 40)
    (send *robot* :head-neck-p :joint-angle 10)
    )
  (if only-upper?
      (progn
        (setq reset-pose-upper #f(0.0 0.0 0.0 0.0 30.0 0.0 55.0 -20.0 -15.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
        (setq pose-under (subseq (send *robot* :angle-vector) 0 12))
        (send *robot* :angle-vector (concatenate float-vector pose-under reset-pose-upper))
        (send *robot* :move-coords (send lleg-coords :translate #f(0 0 100)) (send *robot* :link "LLEG_LINK5")))
    (progn
      (send *robot* :reset-manip-pose)
      (if tilt?
	  (progn
	    (print "tilt torso")
	    (send *robot* :torso-waist-p :joint-angle 10)
	    ;; (send *robot* :head-neck-p :joint-angle 5)
	    (send *robot* :head-neck-p :joint-angle 20)
	    ))
      (start-coords)))
  (if send?
      (progn
	(send-pose movetime :force-move flag-simulator :look-at-object? nil)
	(send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
	(send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords)))
    )
  ;; (send *ri* :hand-reset) ;;実機では必要？choreonoidでは不要？
  (send *ri* :move-gripper :larm 0.0 :wait t)
  (send *ri* :move-gripper :rarm 0.0 :wait t)
  (format t "left or right ?  ~A" (elt (send (car (send *robot* :torso)) :worldpos) 1))
  (if move-foot?
      (if (< 0 (elt (send (car (send *robot* :torso)) :worldpos) 1))
          (progn
            (start-coords)
            (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) :which? :lleg))
        (progn
          (start-coords)
          (move-on-foot rleg-coords lleg-coords (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) :which? :rleg))
        )
    )
  )


(defun low-pass-online
    (target coords arm &key (times 40) (r 0.3) (sleep 10000) (flag nil))
  (let* ((sensor (cond ((equal arm :rarm) :rhsensor) ((equal arm :larm) :lhsensor)))
         (y-data (list (send  (send *robot* (read-from-string (format nil "~A"  sensor)) :copy-worldcoords) :rotate-vector (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm))))
         (y-predict (list (send  (send *robot* (read-from-string (format nil "~A"  sensor)) :copy-worldcoords) :rotate-vector (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm))))
         )
    ;; (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm)
    (dotimes (i times)
      (setq data (send  (send *robot* (read-from-string (format nil "~A"  sensor)) :copy-worldcoords) :rotate-vector (send *ri* :state (read-from-string (format nil ":~A-~A-vector" coords target)) arm)))
      (setq y (v+ (scale r data) (scale (- 1 r) (elt y-predict i))))
      (list-insert y (+ i 1) y-predict)
      (list-insert data (+ i 1) y-data)
      ;; (unix:usleep 1000)
      (unix:usleep sleep)      
      )
    (setq ret (cdr y-predict))
;;    (print (format nil "~A vector through filter = ~A" target ret))
    (setq ret-average (average ret))
    ;;センサからの値は手先センサ座標系,ret-averageの方はとりあえず座標変更した
    (if flag
        (list  (cdr y-data) ret)
      (progn
        (print (format nil "~A  ~A vector through filter and average = ~A" arm target ret-average))
        ret-average)
    )
    )
  )
;; (setq pre-angle (send *robot* :reset-manip-pose))
;; (setq pre-lleg (make-coords :pos #f(0 -100 0)))
(defun set-pre-pose()
  (setq pre-angle (send *robot* :angle-vector))
  (pre-lleg  (send *robot* :lleg :end-coords :copy-worldcoords))
  (list pre-angle pre-lleg))

(defun resume (pre-angle pre-lleg)
  (send *robot* :angle-vector pre-angle)
  (send *robot* :move-coords (send pre-lleg :translate #f(0 0 0)) (send *robot* :lleg :end-coords))
  (load "change_arm_with_ik.l")
  (prepare-simulator))

;;長いものを見つける
(defun get-longest-edge(vertices-list)
  (let ((vertices-list-reverse (reverse vertices-list))
        (longest-len 0))
    (dotimes (i (length vertices-list))
      (dotimes (j (- (length vertices-list) (+ 1 i)))
        (setq len-tmp (norm (v- (elt vertices-list i) (elt vertices-list-reverse j))))
        (if (< longest-len len-tmp)
            (progn
              (setq idx-x i)
              (setq idx-y j) ;;reverseされているがそのままいく
              (setq longest-len len-tmp)))))
    (setq longest-edge (make-line (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))
    longest-edge
    ))

;;重心の左右ごとに平均値をとってそれらを結ぶエッジを得る
(defun get-edge-vector(vertices-list com &key (axis 1))
  (let ((left #f(0 0 0))
        (right #f(0 0 0))
        (left-num 0)
        (right-num 0)
        (com-y (elt com axis)))
    (dotimes (i (length vertices-list))
      (if (< (elt (elt vertices-list i) axis) com-y)
          (progn
            (setq right (v+ right (elt vertices-list i)))
            (setq right-num (+ right-num 1)))
        (progn
          (setq left (v+ left (elt vertices-list i)))
          (setq left-num (+ left-num 1))))
      )
    ;; (format t "left   =~A~%" left)
    ;; (format t "right =~A~%" right)
    
    (setq left-ave (scale (/ 1.0 left-num) left))
    (setq right-ave (scale (/ 1.0 right-num) right))
    (if (equal right-ave #f(-nan -nan -nan))
        (setq right-ave #f(0 0 0)))
    (if (equal left-ave #f(-nan -nan -nan))
        (setq left-ave #f(0 0 0)))
    (format t "left-ave =~A~%" left-ave)
    (format t "right-ave =~A~%" right-ave)    
    ;; (setq y-vector (make-line left-ave right-ave))
    (setq y-vector (normalize-vector (v- right-ave left-ave)))
    (if (equal axis 1)
        (if (< (elt y-vector axis) 0)
            (setq y-vector (scale -1 y-vector)))
      ;;for move-left
      (if (> (elt y-vector axis) 0)
          (setq y-vector (scale -1 y-vector))))
    (format t "edge-vector=~A~%" y-vector)
    y-vector
    ))
        
  
;;plane　オブジェクト(prismなど）をコピーして渡す
;;掴みに行く目標座標
(defun get-target-pose(&key (axis 0) (edge-axis 1));; contact-wall)
  (setq plane-normal-sum #f(0 0 0))
  (setq center-nearhands-sum #f(0 0 0))
  (setq longest-edge-sum #f(0 0 0))
  (setq longest-len-sum 0)
  (setq pos-centroid-sum #f(0 0 0))
  (setq loop-num 2)
  (dotimes (j loop-num)
    (let* ((contact-wall (if (not (make_plane)) (progn (print "can not find any plane") (return-from get-target-pose nil))
                           (make_plane)))
           (idx-x 0) (idx-y 0) (longest-len 0)
           (pos-centroid (elt contact-wall 1))
           (vertices-list (get-min-val (send (car contact-wall) :vertices) (/ (length (send (car contact-wall) :vertices)) 2) :axis axis)) 
           (vertices-list-for-hand-pos-y (get-min-val (send (car contact-wall) :vertices) (/ (* 5 (length (send (car contact-wall) :vertices))) 6) :axis axis) )
           (vertices-list-for-hand-pos-xz (get-min-val (send (car contact-wall) :vertices) (/ (* 1 (length (send (car contact-wall) :vertices))) 3) :axis axis))
           (vertices-list-reverse (reverse vertices-list))
           (plane-normal (elt contact-wall 2)))
      (send *ik-param* :set-object (car contact-wall))
      (format t "plane normal = ~A~%" plane-normal)
      (setq normal-line (make-line #f(0 0 0) plane-normal))

      ;;手先の目標
      (setq sum-y #f(0 0 0)) (setq min-x (elt (car vertices-list-for-hand-pos-y) axis)) (setq num-y 0)
      (setq min-tmp 0) (setq min-idx 0)
      (setq max-tmp 0) (setq max-idx 0)

      (dotimes (i (length vertices-list-for-hand-pos-y))
        (if (< (elt (elt vertices-list-for-hand-pos-y i) axis) (+ min-x 500))
            (progn
              (if (< (elt (elt vertices-list-for-hand-pos-y i) 1) min-tmp)
                  (progn 
                    (setq min-tmp (elt (elt vertices-list-for-hand-pos-y i) 1))
                    (setq min-idx i)
                    ))
              (if (< max-tmp (elt (elt vertices-list-for-hand-pos-y i) 1))
                  (progn 
                    (setq max-tmp (elt (elt vertices-list-for-hand-pos-y i) 1))
                    (setq max-idx i)
                    ))
              (setq sum-y (v+ sum-y (elt vertices-list-for-hand-pos-y i)))
              (setq num-y (+ num-y 1))
              )
          ))
      (setq pos-tmp-y (scale 0.5 (v+ (elt vertices-list-for-hand-pos-y min-idx) (elt vertices-list-for-hand-pos-y max-idx))))
      
      (setq sum-xz #f(0 0 0)) (setq num-xz 0)
      (setq min-x (elt (car vertices-list-for-hand-pos-xz) axis))
      (dotimes (i (length vertices-list-for-hand-pos-xz))
        (if (< (elt (elt vertices-list-for-hand-pos-xz i) axis) (+ min-x 100))
            (progn
              (setq sum-xz (v+ sum-xz (elt vertices-list-for-hand-pos-xz i)))
              (setq num-xz (+ num-xz 1)))))
      (setq pos-tmp-xz (scale (/ 1.0 num-xz) sum-xz))
      
      (setq center-nearhands-inloop (float-vector (elt pos-tmp-xz 0) (elt pos-tmp-y 1) (elt pos-tmp-xz 2)))
      (format t "mid-pos near hands in loop= ~A~%" center-nearhands-inloop)
      (setq center-nearhands-sum (v+ center-nearhands-sum center-nearhands-inloop))
      (setq longest-edge-sum (v+ longest-edge-sum (get-edge-vector vertices-list center-nearhands-inloop :axis edge-axis))) ;;vector
      (setq longest-len-sum (+ (send (get-longest-edge vertices-list) :length)))
      (setq pos-centroid-sum (v+ pos-centroid-sum pos-centroid))
      (setq plane-normal-sum (v+ plane-normal-sum plane-normal))
      (unix::usleep 100000)
      ))
  (setq plane-normal (scale (/ 1.0 loop-num) plane-normal-sum))
  (setq longest-edge (scale (/ 1.0 loop-num) longest-edge-sum))
  (setq longest-len (* (/ 1.0 loop-num) longest-len-sum))
  (setq pos-centroid (scale (/ 1.0 loop-num) pos-centroid-sum))
  (setq center-nearhands (scale (/ 1.0 loop-num) center-nearhands-sum))
  (format t "mid-pos near hands= ~A~%" center-nearhands)
  ;;edgeを得る
  ;; (setq longest-edge (get-longest-edge vertices-list))
  ;; (setq longest-edge (get-edge-vector vertices-list pos :axis edge-axis))
  ;; (setq longest-len (send (get-longest-edge vertices-list) :length))
  ;; (format t "longest-len = ~A~%" longest-len)
  
  (setq y-vec  plane-normal)
  ;; (setq z-vec (v- (elt vertices-list idx-x) (elt vertices-list-reverse idx-y)))
  ;; (setq z-vec (send longest-edge :direction))
  (setq z-vec longest-edge)
  (setq x-vec (v* y-vec z-vec))
  ;;x-vecは正面方向
    (if (< (elt x-vec 0) 0)
        (setq x-vec (scale -1 x-vec)))
    (if (< (elt y-vec 2) 0)
        (setq y-vec (scale -1 y-vec)))

    ;;move-left
    (if (equal axis 1)
        (progn 
          (if (< (elt x-vec 1) 0)
              (setq x-vec (scale -1 x-vec)))))
    (setq x-vec (normalize-vector x-vec))
    (setq y-vec (normalize-vector y-vec))
    (setq z-vec (normalize-vector z-vec))
    ;; (format t "plane-normal = ~A~%" plane-normal) (format t "x-vec = ~A~%" x-vec) (format t "y-vec = ~A~%" y-vec) (format t "z-vec = ~A~%" z-vec)    
    (list x-vec y-vec z-vec center-nearhands longest-len pos-centroid)
    ;; (list x-vec y-vec z-vec pos-centroid longest-len pos-centroid)    ;;big board
    )
  
;;掴みに行く目標座標　微調整
;;pos-shift=0 にして返る返り値のうちどちらの手を使うかを選べば良い
(ros::roseus "hand_targets")
(ros::advertise "/larm_target" geometry_msgs::PointStamped 1)
(ros::advertise "/rarm_target" geometry_msgs::PointStamped 1)
(defun auto-set-reaching-targets
    (&key (pos-shift 300) (larm-shift #f(-30 0 0)) (rarm-shift #f(-30 0 0)) (rot-z 20) (rot-x 0) (rot-y 0) (center-pos #f(0 0 0)) (axis 0) (edge-axis 1))
  ;; (setq ret (get-target-pose)) ;;0124 coment
  (setq ret (get-target-pose :axis axis :edge-axis edge-axis)) ;;

  (setq x-vec (elt ret 0))
  (setq y-vec (elt ret 1))
  (setq z-vec (elt ret 2))
  (setq pos   (elt ret 3))
  (setq longest-len (elt ret 4))
  ;; (if  (equal axis 0)
  ;;     (setq pos (float-vector (elt pos 0) (elt center-pos 1) (elt pos 2)))) ;;reach auto 

  ;; (setq pos-larm (v+ pos (float-vector 0 pos-shift 0)))
  (if (< (elt y-vec 2) 0)
      (setq y-vec-larm (scale -1 y-vec))
    (setq y-vec-larm  y-vec))
  (if (equal axis 0)
      (if (> (elt z-vec 1) 0)
	  (setq z-vec-larm (scale -1 z-vec))
	(setq z-vec-larm z-vec))
    (if (< (elt z-vec 0) 0)
	(setq z-vec-larm (scale -1 z-vec))
      (setq z-vec-larm z-vec)))
  (format t "z-vec = ~A~%" z-vec)
  ;; (setq pos-rarm (v+ pos (float-vector 0 (- 0 pos-shift) 0)))
  
  (if (> (elt y-vec 2) 0)
      (setq y-vec-rarm (scale -1 y-vec))
    (setq y-vec-rarm y-vec))
  (if (equal axis 0)
      (if (< (elt z-vec 1) 0)
	  (setq z-vec-rarm (scale -1 z-vec))
	(setq z-vec-rarm z-vec))
    (if (> (elt z-vec 0) 0)
	(setq z-vec-rarm (scale -1 z-vec))
      (setq z-vec-rarm z-vec))   ) 
  (format t "z-vec = ~A~%" z-vec)
  (setq rot-larm (transpose
                  (matrix x-vec y-vec-larm z-vec-larm)))
  (setq rot-rarm (transpose
                  (matrix x-vec y-vec-rarm z-vec-rarm)))
  
  ;; (format t "pos-larm = ~A~%" pos-larm)
  ;; (format t "rot-larm = ~A~%" rot-larm)
  ;; (format t "pos-rarm = ~A~%" pos-rarm)
  ;; (format t "rot-rarm = ~A~%" rot-rarm)
  (setq coords1 (make-coords))
  ;; (setq larm-coords (make-coords :pos pos-larm :rpy (elt (rpy-angle rot-larm)  0)))
  ;; (setq rarm-coords (make-coords :pos pos-rarm :rpy (elt (rpy-angle rot-rarm)  0)))
  (format t "pos-shift = ~A~%" pos-shift)
;;lleg com dist 引くのは間違っている？
;; (setq lleg-com-dist (v- (send *robot* :lleg :end-coords :worldpos) (midpoint 0.5 (send *robot* :lleg :end-coords :worldpos) (send *robot* :rleg :end-coords :worldpos))))
  (setq lleg-com-dist #f(0 0 0))
  (format t "lleg-com-dist = ~A~%" lleg-com-dist)
  ;;傾いたエッジ対応
  (setq larm-coords (make-coords :pos (v+ (v- pos lleg-com-dist) (scale (* -1 pos-shift) z-vec-larm)) :rpy (elt (rpy-angle rot-larm)  0)))
  (setq rarm-coords (make-coords :pos (v+ (v- pos lleg-com-dist) (scale (* -1 pos-shift) z-vec-rarm)) :rpy (elt (rpy-angle rot-rarm)  0)))
  ;; (format t "ram-coords = ~A~%" rarm-coords)
  ;; (setq larm-coords (send larm-coords :translate (float-vector 0 0 (- 0 pos-shift)) :local))
  ;; (setq rarm-coords (send rarm-coords :translate (float-vector 0 0 (+ 0 pos-shift)) :local))
  (format t "ik larm-coords = ~A~%" larm-coords)
  (format t "ik rarm-coords = ~A~%" rarm-coords)
  ;;少し回転
  (setq larm-coords (send larm-coords :rotate (deg2rad (- 0 rot-z)) :z))
  (setq rarm-coords (send rarm-coords :rotate (deg2rad rot-z) :z))
  ;;for remove left 
  (setq larm-coords (send larm-coords :rotate (deg2rad (- 0 rot-x)) :x))
  (setq rarm-coords (send rarm-coords :rotate (deg2rad rot-x) :x))
  (setq larm-coords (send larm-coords :rotate (deg2rad (- 0 rot-y)) :y))
  (setq rarm-coords (send rarm-coords :rotate (deg2rad rot-y) :y))
  (format t "rot y ~A~%" rot-y)
  (format t "ik larm-coords = ~A~%" larm-coords)
  (format t "ik rarm-coords = ~A~%" rarm-coords)
    (format t "ik rarm-shift = ~A~%" rarm-shift)  
  (setq larm-coords (send larm-coords :translate larm-shift :local))
  (setq rarm-coords (send rarm-coords :translate rarm-shift :local))
  (format t "ik larm-coords = ~A~%" larm-coords)
  (format t "ik rarm-coords = ~A~%" rarm-coords)

  (setq larm-coords-draw  (instance arrow-object :init))
  (send larm-coords-draw :newcoords larm-coords)
  (setq rarm-coords-draw  (instance arrow-object :init))
  (send rarm-coords-draw :newcoords rarm-coords)
  (setq normal-line (make-line (v+ com #f(0 0 0)) (v+ com (scale 300 normal-vec))))
  (objects (list (car (make_plane)) coords1 larm-coords-draw rarm-coords-draw normal-line))

  ;;rviz visualize reaching point 
  (progn
    (setq larm-pos-pub (instance geometry_msgs::PointStamped :init))
    (setq rarm-pos-pub (instance geometry_msgs::PointStamped :init))
    (setq l-point (send larm-coords :worldpos))
    (setq r-point (send rarm-coords :worldpos))
    (send larm-pos-pub :header :stamp (ros::time-now))
    (send larm-pos-pub :header :frame_id "lleg_end_coords")
    (send larm-pos-pub :point :x (* 0.001 (elt l-point 0))) (send larm-pos-pub :point :y (* 0.001 (elt l-point 1))) (send larm-pos-pub :point :z (* 0.001 (elt l-point 2)))
    (send rarm-pos-pub :header :stamp (ros::time-now))
    (send rarm-pos-pub :header :frame_id "lleg_end_coords")
    (send rarm-pos-pub :point :x (* 0.001 (elt r-point 0))) (send rarm-pos-pub :point :y (* 0.001 (elt r-point 1))) (send rarm-pos-pub :point :z (* 0.001 (elt r-point 2)))
    (format t "l-point = ~A~%" l-point)
    (ros::publish "/larm_target" larm-pos-pub)
    (ros::publish "/rarm_target" rarm-pos-pub)
    )
  (list larm-coords rarm-coords pos)
  )

;;0214
;;rarm-point,larm-point のそれぞれから近いエッジを見つけてedge-pos,edge-vecを用いて位置、姿勢を計算する
;;エッジはmin-lenより長いものを選ぶ
;;return larm rarm centroid
(defun auto-set-reaching-targets-each-hand(&key (pos-shift 600) (loop-num 2) (rot-x 0) (rot-y 0) (rot-z 20) (larm-shift #f(-30 0 0)) (rarm-shift #f(-30 0 0)) (min-len 40))
  (let*
      ((rarm-point (v+ (send (send *robot* :link "RARM_LINK1") :worldpos) (float-vector 400 (* -1 (/ pos-shift 2)) -500)))
       (*rarm-point* (instance arrow-object :init))
       (larm-point (v+ (send (send *robot* :link "LARM_LINK1") :worldpos) (float-vector 400 (/ pos-shift 2) -500)))
       (*larm-point* (instance arrow-object :init)))

    (send *rarm-point* :newcoords (make-coords :pos rarm-point))
    (send *larm-point* :newcoords (make-coords :pos larm-point))
    (setq plane-normal-sum #f(0 0 0)
          rarm-edge-vec-sum #f(0 0 0)
          larm-edge-vec-sum #f(0 0 0)
          rarm-edge-pos-sum #f(0 0 0)
          larm-edge-pos-sum #f(0 0 0))
    
    (dotimes (i loop-num)
      (setq fc (make_plane))
      (setq small-distance-lis (get-min-val (mapcar #'(lambda(v) (list (send v :distance rarm-point) (send v :length) v)) (send (car fc) :edges)) 12)
            ;; rarm-edge-tmp (cadr (car small-distance-lis))
            length-t-nil-list (mapcar #'(lambda(l) (< min-len (elt l 1) )) small-distance-lis) 
            rarm-edge-tmp (elt (elt small-distance-lis (- (length small-distance-lis) (length (member t length-t-nil-list)))) 2)
            rarm-edge-vec-tmp (normalize-vector (send rarm-edge-tmp :direction))
            rarm-edge-pos-tmp (send rarm-edge-tmp :point (if (and (< 0 (send rarm-edge-tmp :foot rarm-point)) (< (send rarm-edge-tmp :foot rarm-point) 1)) (send rarm-edge-tmp :foot rarm-point) 0.5)))
      (setq small-distance-lis (get-min-val (mapcar #'(lambda(v) (list (send v :distance larm-point) (send v :length) v)) (send (car fc) :edges)) 12)
            ;; larm-edge-tmp (cadr (car small-distance-lis))
            length-t-nil-list (mapcar #'(lambda(l) (< min-len (elt l 1) )) small-distance-lis)
            larm-edge-tmp (elt (elt small-distance-lis (- (length small-distance-lis) (length (member t length-t-nil-list)))) 2)
            larm-edge-vec-tmp (normalize-vector (send larm-edge-tmp :direction))
            larm-edge-pos-tmp (send larm-edge-tmp :point (if (and (< 0 (send larm-edge-tmp :foot larm-point)) (< (send larm-edge-tmp :foot larm-point) 1)) (send larm-edge-tmp :foot larm-point) 0.5)))
      (setq plane-normal-tmp (elt fc 2))

      ;;for average
      (setq plane-normal-sum (v+ plane-normal-sum (if (< 0 (elt plane-normal-tmp 2)) plane-normal-tmp (scale -1 plane-normal-tmp)))
            rarm-edge-vec-sum (v+ rarm-edge-vec-sum (if (< 0 (elt rarm-edge-vec-tmp 1)) rarm-edge-vec-tmp (scale -1 rarm-edge-vec-tmp))) ;;方向調整
            larm-edge-vec-sum (v+ larm-edge-vec-sum (if (< (elt larm-edge-vec-tmp 1) 0) larm-edge-vec-tmp (scale -1 larm-edge-vec-tmp)))
            rarm-edge-pos-sum (v+ rarm-edge-pos-sum rarm-edge-pos-tmp)
            larm-edge-pos-sum (v+ larm-edge-pos-sum larm-edge-pos-tmp)
            ))

    ;;average
    (setq plane-normal (scale (/ 1.0 loop-num) plane-normal-sum)
            rarm-edge (scale (/ 1.0 loop-num) rarm-edge-vec-sum)
            larm-edge (scale (/ 1.0 loop-num) larm-edge-vec-sum)
            rarm-pos (scale (/ 1.0 loop-num) rarm-edge-pos-sum)
            larm-pos (scale (/ 1.0 loop-num) larm-edge-pos-sum))
      
    (setq y-vec-larm plane-normal
          y-vec-rarm  (scale -1 plane-normal)
          z-vec-rarm  rarm-edge
          z-vec-larm  larm-edge
          x-vec-rarm  (normalize-vector (v* y-vec-rarm z-vec-rarm))
          x-vec-larm  (normalize-vector (v* y-vec-larm z-vec-larm)))
      
      (setq rot-rarm (transpose
                      (matrix x-vec-rarm y-vec-rarm z-vec-rarm)))
      (setq rarm-coords (make-coords :pos rarm-pos :rpy (elt (rpy-angle rot-rarm)  0)))
      (setq rarm-coords (send rarm-coords :translate rarm-shift :local))
      (setq rarm-coords (send rarm-coords :rotate (deg2rad rot-z) :z))
      (setq rarm-coords (send rarm-coords :rotate (deg2rad rot-x) :x))
      (setq rarm-coords (send rarm-coords :rotate (deg2rad rot-y) :y))
      
      (setq rot-larm (transpose
                      (matrix x-vec-larm y-vec-larm z-vec-larm)))
      (setq larm-coords (make-coords :pos larm-pos :rpy (elt (rpy-angle rot-larm)  0)))
      (setq larm-coords (send larm-coords :translate larm-shift :local))
      (setq larm-coords (send larm-coords :rotate (deg2rad (- 0 rot-z)) :z))
      (setq larm-coords (send larm-coords :rotate (deg2rad (- 0 rot-x)) :x))
      (setq larm-coords (send larm-coords :rotate (deg2rad (- 0 rot-y)) :y))

      (setq *rarm-coords*  (instance arrow-object :init))
      (send *rarm-coords* :newcoords rarm-coords)
      (setq *larm-coords*  (instance arrow-object :init))
      (send *larm-coords* :newcoords larm-coords)
      (setq *normal-line* (make-line (v+ com #f(0 0 0))
                                   (v+ com (scale 300 plane-normal))))
      (objects (list (car fc) *larm-coords* *rarm-coords* *normal-line* *rarm-point* *larm-point* rarm-edge-tmp larm-edge-tmp))

   ;; rviz visualize reaching point 
      (progn
        (setq larm-pos-pub (instance geometry_msgs::PointStamped :init))
        (setq rarm-pos-pub (instance geometry_msgs::PointStamped :init))
        (setq l-point (send larm-coords :worldpos))
        (setq r-point (send rarm-coords :worldpos))
        (send larm-pos-pub :header :stamp (ros::time-now))
        (send larm-pos-pub :header :frame_id "lleg_end_coords")
        (send larm-pos-pub :point :x (* 0.001 (elt l-point 0))) (send larm-pos-pub :point :y (* 0.001 (elt l-point 1))) (send larm-pos-pub :point :z (* 0.001 (elt l-point 2)))
        (send rarm-pos-pub :header :stamp (ros::time-now))
        (send rarm-pos-pub :header :frame_id "lleg_end_coords")
        (send rarm-pos-pub :point :x (* 0.001 (elt r-point 0))) (send rarm-pos-pub :point :y (* 0.001 (elt r-point 1))) (send rarm-pos-pub :point :z (* 0.001 (elt r-point 2)))
        (format t "l-point = ~A~%" l-point)
        (ros::publish "/larm_target" larm-pos-pub)
        (ros::publish "/rarm_target" rarm-pos-pub)
        )
      (list x-vec-rarm y-vec-rarm z-vec-rarm (send rarm-coords :worldpos) x-vec-larm y-vec-larm z-vec-larm (send larm-coords :worldpos))
      (list larm-coords rarm-coords (send (car fc) :centroid))
      )
  )

(defun get-half-plane(obj &key (cut-rate 0.5))
  (setq origin-obj (copy-object obj))
  (setq origin-vertices (send (car (send origin-obj :faces)) :vertices))
  (setq com (send origin-obj :centroid))
  (format t "com = ~A~%" com)
  (setq len-lis (length origin-vertices))
  ;; (setq cub (make-cube (* (/ 1.0 cut-rate) (abs (elt com 0))) 5000 5000))
  (setq cub (make-cube (* (* 2 (- 1 cut-rate)) (* 2 (abs (elt com 0)))) 5000 5000))
  (send cub :translate (scale 2.0 com))
  (setq sub (body- origin-obj cub))
  (send sub :set-color (float-vector 1 0 0))  
  (objects (list  sub ))
  sub)

(defun board-holding-pose(&key (send? nil) (fix-contact? t) (look-at-object? nil))

  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (send *robot* :angle-vector  #f(-2.98947 -11.1725 -59.0636 8.42639 24.4303 12.4009 7.00572 10.9221 -59.3257 7.19791 25.9469 -11.8133 0.570165 3.0 1.84699 0.603257 15.0 37.0 35.8508 -131.711 -28.0764 -23.8558 -27.5811 24.8872 -0.448495 -37.0 35.6619 131.041 28.564 -24.874 27.783 -24.4164 0.215248))
  ;; (send *robot* :angle-vector
  ;; 	#f(-1.32255 -6.24016 -29.6795 33.8312 -11.7631 7.52237 0.898833 15.4077 -26.725 24.6561 -5.76486 -14.2255 1.15928 3.0 -0.655261 2.4144 16.5573 37.0 45.5976 -109.064 -30.8867 -64.1641 -26.6447 9.51422 22.1128 -37.0 41.1018 104.469 28.6239 -56.9815 20.0663 -14.0334 11.5481))
  ;; (send *robot* :angle-vector  #f(-1.32255 -6.24016 -29.6795 33.8312 -11.7631 7.52237 0.898833 15.4077 -26.725 24.6561 -5.76486 -14.2255 1.15928 3.0 -0.655261 2.4144 0.0 37.0 45.5976 -109.064 -30.8867 -64.1641 -26.6447 9.51422 22.1128 -37.0 41.1018 104.469 28.6239 -56.9815 20.0663 -14.0334 11.5481))
  ;;from reaching-pose (heave #f(-400 -1000 0) #f(-400 1000 0) :reset? t :fix-contact? 2 :wait? t  :rleg-pos-shift #f(200 0 0) :lleg-pos-shift #f(200 0 0) :look-at-object? t)
  ;; (send *robot* :angle-vector   #f(0.497866 -11.5794 -10.9614 38.9877 -18.0864 11.1527 -0.660753 10.6076 -10.7331 38.745 -18.0875 -10.9056 0.0 3.0 0.065955 -0.009049 20.0 37.0 34.7493 -74.2251 -25.5205 -90.7107 6.29055 1.54029 30.4263 -37.0 36.0901 76.0089 25.7384 -90.2711 -3.90134 -2.16365 30.4717))
  (send *robot* :angle-vector #f(0.198761 -11.1901 -15.2719 36.987 -13.1589 10.9564 -0.398351 10.8076 -15.1034 36.9009 -13.2446 -10.8891 0.0 3.0 -0.026228 -0.005043 18.4018 37.0 38.4206 -88.7584 -30.2222 -80.624 -3.43998 -3.26935 26.4778 -37.0 39.2722 90.1039 30.227 -79.7785 4.40195 2.48036 26.5397))
  (send *robot* :head-neck-p :joint-angle 10)
  (send *robot* :move-coords (make-coords :pos #f(0 300 0)) (send *robot* :lleg :end-coords))
  (objects (list *robot* ))
  (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?  )
  (setq zmp-vector (send *ri*  :zmp-vector))
  (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?)
  (if send?
      (progn 
        (if (not fix-contact?)
            (progn
              (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
              (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t))
  	  )
        (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?  )))
  )

;;IKを解かない方
(defun reaching-pose (&key (send? nil) (fix-contact? nil) (look-at-object? nil))
  ;; (send *ri* :start-impedance :arms)
  ;; (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  ;; (send *ri* :start-reference-force-updater :arms)
  ;; (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  (my-init-pose :move-foot? nil :send? send?)
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  ;;踏み出しで落ちやすい
  ;; (send *robot* :angle-vector #f(-10.6248 -6.65663 -106.861 50.9002 3.98799 12.4148 10.6723 6.7806 -106.801 51.0553 3.75381 -12.5862 -0.183996 27.0 0.021731 0.021981 18.0 37.0 -5.52897 -49.7907 -25.152 -80.696 57.979 -29.8714 13.8606 -37.0 -5.22003 50.1071 25.3023 -80.8198 -57.8915 29.7632 13.9217))
  ;;低
  (send *robot* :angle-vector #f(-8.58676 -4.0563 -95.8534 56.6709 0.402954 9.52003 8.99794 14.9015 -94.6692 49.0149 5.26972 -16.1815 -6.0 30.0 0.300236 -8.04015 -40.0 36.7888 -14.5601 -49.5236 -14.4544 -78.1538 49.999 -18.7491 20.2343 -36.9302 21.5486 79.1799 31.4005 -83.5811 -33.5652 19.8654 24.6522))
  ;;中
  ;; (send *robot* :angle-vector #f(-2.07669 -1.21954 -53.9677 57.0 -12.1277 6.02715 1.97674 22.802 -51.9767 51.175 -9.06998 -18.3169 -6.0 23.2969 3.39918 -1.38038 0.158868 37.0 -4.49404 -38.7688 -4.50152 -72.5887 42.9935 -8.56576 22.6918 -37.0 27.0826 67.6348 19.6725 -77.6165 -25.9171 8.7954 18.0071))
  (send *robot* :move-coords (make-coords :pos #f(0 280 0)) (send *robot* :lleg :end-coords))
  ;; (send *robot* :head-neck-p :joint-angle -40)
  (objects (list *robot* ))
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact? :look-at-object? t)
  ;; (send *robot* :head-neck-p :joint-angle -40)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 4000)  
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn 
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))
  ;;       (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?  )))
)

;;reach 1203
;;IKを解く方
(defun reach-new()
  ;; (board-holding-pose)
  (send *ri* :start-impedance :arms)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  
  (setq rarm-coords (make-coords :pos (float-vector 360 -310 300) :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad -90))))
  (setq larm-coords (make-coords :pos (float-vector 360  310 300)  :rpy (float-vector (deg2rad 0) (deg2rad  30) (deg2rad  90))))
  (reach :rarm-coords rarm-coords :larm-coords larm-coords :send? t :avoid-collision? t :apply-external-force? t :rleg-pos-shift #f(0 -100 -10) :lleg-pos-shift #f(0 100 -10) :look-at-object? t)
  )

(defun decide-move-dir(normal-vec-save normal-vec l-pos-diff r-pos-diff)
  (let* ((l-omega (v* normal-vec-save l-pos-diff))
         (r-omega (v* normal-vec-save r-pos-diff))
         (l-theta (abs (asin (/ (norm l-omega) (* (norm normal-vec-save) (norm l-pos-diff)))))) ;;rad
         (r-theta (abs (asin (/ (norm r-omega) (* (norm normal-vec-save) (norm r-pos-diff))))))
         (l-theta-next (* (/ l-unit l-unit-save) l-theta))
         (r-theta-next (* (/ r-unit r-unit-save) r-theta))
         (l-rot-mat (rotation-matrix l-theta-next l-omega))
         (r-rot-mat (rotation-matrix r-theta-next r-omega))
         (l-move-dir (transform l-rot-mat normal-vec))
         (r-move-dir (transform r-rot-mat normal-vec))
         )
    (format t "~%~%normal-vec-save = ~A~%" normal-vec-save)
    (format t "l-pos-diff          = ~A~%" l-pos-diff)
    (format t "r-pos-diff          = ~A~%" r-pos-diff)
    (format t "l-omega = ~A~%" (normalize-vector l-omega))
    (format t "r-omega = ~A~%" (normalize-vector r-omega))
    
    (format t "r-theta deg      = ~A~%" (rad2deg r-theta))
    (format t "l-theta deg      = ~A~%" (rad2deg l-theta))    
    (format t "r-theta-next deg = ~A~%" (rad2deg r-theta-next))
    (format t "l-theta-next deg = ~A~%" (rad2deg l-theta-next))
    (format t "r-move-dir = ~A~%" r-move-dir)
    (format t "l-move-dir = ~A~%~%" l-move-dir)
    
    (list l-move-dir r-move-dir (rotation-matrix (*  0.5 l-theta-next) l-omega) (rotation-matrix (*  0.5 r-theta-next) r-omega ))
    ))

(ros::advertise "/normal_start" geometry_msgs::PointStamped 1)
(ros::advertise "/normal_end" geometry_msgs::PointStamped 1)

(defun heave-with-sensor
    (larm-move-offset
     rarm-move-offset
     times
     &key  (fix-contact? 2) (send? t) (wait? t) (anti-slip-rate 1.0) (check? nil) (target "board") 
     (reset? t) (apply-external-force? t) (avoid-collision? t)
     (rleg-pos-shift #f(80 -20 0))
     (lleg-pos-shift #f(80 20 0))
     (look-at-object? t)
     ;; (reaching-offset (list #f(-100 0 0) #f(-100 0 0)))
     (fix-hand? nil)
     (rot-z 0)
     (first-lift (list 100 100))
     (l-omega #f(0 1 0))
     (r-omega #f(0 1 0))
     (arm "both")
     (x-plus #f(0.1 0 0))
     (thre-lnum 1000)
     (thre-rnum 1000)
     )
  (reset-coords)
  (setq normal-vec-diff #f(0 0 0))
  (setq l-normal-vec-scaled-diff #f(0 0 0))
  (setq r-normal-vec-scaled-diff #f(0 0 0))
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :move-coords (send *coords* :get-lleg-coords) (send *robot* :lleg :end-coords))
  (setq l-act-coords-save (send *robot* :larm :end-coords :copy-worldcoords))
  (setq r-act-coords-save (send *robot* :rarm :end-coords :copy-worldcoords))
  (format t "l-act-coords-save init =~A~%" l-act-coords-save)
  (format t "r-act-coords-save init =~A~%" r-act-coords-save)

  ;;平面が見つからない時
  (setq neck-angle  (send *robot* :head-neck-p :joint-angle))
  (while (not (make_plane))
    (setq neck-angle (+ neck-angle 5))
    (format t "look down angle = ~A~%" neck-angle)
    (send *robot* :head-neck-p :joint-angle neck-angle)
    (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    )
  (format t "find neck angle = ~A~%" neck-angle)
  ;; (send *ri* :start-log)
  ;; (setq lis (list #f(0.30177 0.00669 0.953357)  #f(0.353907 0.000234 0.93528) #f(0.302493 0.003507 0.953145)))
  (dotimes (i times)
    (reset-coords)
    (if (= i 0)
        (progn
          (if (< (abs larm-move-offset) (abs (elt first-lift 0)))
              ;; (setq l-unit (* larm-move-offset (/ larm-move-offset (abs larm-move-offset))))
              (setq l-unit larm-move-offset )
            (setq l-unit (* (elt first-lift 0) (/ larm-move-offset (abs larm-move-offset)))))
          (if (< (abs rarm-move-offset) (abs (elt first-lift 1)))
              ;; (setq r-unit (* rarm-move-offset (/ rarm-move-offset (abs rarm-move-offset))))
              (setq r-unit rarm-move-offset)
            (setq r-unit (* (elt first-lift 1) (/ rarm-move-offset (abs rarm-move-offset)))))
          )
      (progn
        (setq l-unit (* (/ 1.0 (- times 1)) (- larm-move-offset (* (elt first-lift 0) (/ larm-move-offset (abs larm-move-offset))))))
        (setq r-unit (* (/ 1.0 (- times 1)) (- rarm-move-offset (* (elt first-lift 1) (/ rarm-move-offset (abs rarm-move-offset))))))))
    (format t "l-unit = ~A~%" l-unit)
    (format t "r-unit = ~A~%" r-unit)

    (setq fc (make_plane))
    (send *ik-param* :set-object (car fc))
    ;; (setq coords-list (auto-set-reaching-targets :pos-shift 0 :larm-shift #f(0 0 0) :rarm-shift #f(0 0 0) :rot-z rot-z))
    (setq normal-vec (elt fc 2))
    (if (< (elt normal-vec 2) 0)
        (setq normal-vec (scale -1.0 normal-vec)))
    ;; (setq normal-vec (elt lis i))
    (format t "normal-vec =~A~%" normal-vec)
    ;;new version 0107
    (setq l-move-dir  (normalize-vector (v+ normal-vec x-plus)))
    (setq r-move-dir  (normalize-vector (v+ normal-vec x-plus )))
    (setq l-theta-next (deg2rad 10))
    (setq r-theta-next (deg2rad 10))

    (setq l-pos-diff normal-vec)
    (setq r-pos-diff normal-vec)
     
    (if (< 0 i)
        (progn
          (setq move-dir (decide-move-dir normal-vec-save normal-vec l-pos-diff r-pos-diff))
          (setq l-move-dir (elt move-dir 0)
                r-move-dir (elt move-dir 1)
                l-rot (elt move-dir 2)
                r-rot (elt move-dir 3))
          ))

    (setq lnum-pre (send (one-shot-subscribe "/larm/num" std_msgs::Int32 :timeout 5000) :data)
          rnum-pre (send (one-shot-subscribe "/rarm/num" std_msgs::Int32 :timeout 5000) :data))
    (print "ok")

    (setq larm-coords (make-coords :pos (v+ (send *robot* :larm :end-coords :worldpos) (scale l-unit l-move-dir)) :rot (m* (send *robot* :larm :end-coords :worldrot) l-rot)))
    (setq rarm-coords (make-coords :pos (v+ (send *robot* :rarm :end-coords :worldpos) (scale r-unit r-move-dir)) :rot (m* (send *robot* :rarm :end-coords :worldrot) r-lot)))

    (format t "larm-coords     = ~A~%" (send *robot* :larm :end-coords :copy-worldcoords))
    (format t "rarm-coords     = ~A~%" (send *robot* :rarm :end-coords :copy-worldcoords))
    (format t "larm-coords next= ~A~%" larm-coords)
    (format t "rarm-coords next= ~A~%" rarm-coords)
    (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
    (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
    (setq larm-force (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
    (setq rarm-force (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))

    ;;立ち位置変更の切り替え
    (setq waist-z-thre 700)
    ;; (setq hand-x-thre 500)
    ;; (setq hand-x-thre 500)
    (setq hand-x-thre 600)
    ;; (setq hand-z-thre 1200)
    ;; (setq hand-z-thre 1000)
    (setq hand-z-thre (elt (send (car (send *robot* :links)) :worldpos) 2))
    (format t "waist z = ~A  limit =~A~%" (elt (send (car (send *robot* :links)) :worldpos) 2) waist-z-thre)
    (format t "hand  x = ~A  limit =~A~%" (-(elt (send larm-coords :worldpos) 0) (elt (send (send *coords* :get-lleg-coords) :worldpos) 0)) hand-x-thre)
    (format t "hand  z = ~A  limit =~A~%"  (elt (send larm-coords :worldpos) 2) hand-z-thre)
    (setq heave? nil)
    (if (< 0 i)
        (progn
          (if (and (< waist-z-thre (elt (send (car (send *robot* :links)) :worldpos) 2))
                   (or (< hand-z-thre (elt (send larm-coords :worldpos) 2)) (< hand-x-thre (-(elt (send larm-coords :worldpos) 0) (elt (send (send *coords* :get-lleg-coords) :worldpos) 0)))));;0109 or to and
              (progn 
                (setq fix-contact? nil)
                (setq fix-hand? nil)
                (setq wait? nil)
		(setq heave? t))
	    (progn 
                (setq fix-contact? 2)
                (setq fix-hand? nil)
                (setq wait? t)
		(setq heave? nil))
	    )
          ))
          ;; (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents (scale -1 rarm-force) :initial-element 0))
    ;; (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents (scale -1 larm-force) :initial-element 0))
    (setq rarm-force #f(0 0 -40))
    (setq larm-force #f(0 0 -40))
    (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents (scale +1 rarm-force) :initial-element 0))
    (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents (scale +1 larm-force) :initial-element 0))    
          ;; ))
    (send *coords* :set-rarm-target-coords rarm-coords)
    (send *coords* :set-larm-target-coords larm-coords)
    (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
    (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
    (send *ik-param* :set-fix-contact? fix-contact?)
    (send *ik-param* :set-apply-external-force? apply-external-force?)
    (send *ik-param* :set-wait? wait?)
    (send *ik-param* :set-rleg-pos-shift rleg-pos-shift)
    (send *ik-param* :set-lleg-pos-shift lleg-pos-shift)
    ;; (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents rarm-force :initial-element 0))
    ;; (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents larm-force :initial-element 0))
    (setq half-wrench (scale 0.5 (v+ (send *ik-param* :get-rarm-wrench) (send *ik-param* :get-larm-wrench))))
    (send *ik-param* :set-rleg-wrench   (v+ (send *ik-param* :get-rleg-wrench-offset) (scale -1 half-wrench)))
    (send *ik-param* :set-lleg-wrench   (v+ (send *ik-param* :get-lleg-wrench-offset) (scale -1 half-wrench)))
    (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))

    ;; (send *ik-param* :set-rleg-wrench #f(0 0 0 0 0 0))
    ;; (send *ik-param* :set-lleg-wrench #f(0 0 0 0 0 0))
    (setq ret (ik-opt rarm-coords larm-coords  :fix-contact? fix-contact?  :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift  :kin-task-value-thre 0.15 :eom-trans-task-value-thre 30 :heave? heave? :arm arm))
    (send *robot-env* :dissoc-root-virtual)
    ;;とき直し
    (if (equal ret nil)
        (progn
          (reset-coords)
          (format t "change hand external force~%")
          (setq larm-force (v+ larm-force #f(0 0 -10)))
          (setq rarm-force (v+ rarm-force #f(0 0 -10)))
          ;; (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents rarm-force :initial-element 0))
          ;; (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents larm-force :initial-element 0))
          ;; (setq half-wrench (scale 0.5 (v+ (send *ik-param* :get-rarm-wrench) (send *ik-param* :get-larm-wrench))))
          ;; (send *ik-param* :set-rleg-wrench   (v+ (send *ik-param* :get-rleg-wrench-offset) half-wrench))
          ;; (send *ik-param* :set-lleg-wrench   (v+ (send *ik-param* :get-lleg-wrench-offset) half-wrench))
	  (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents (scale 1 rarm-force) :initial-element 0))
      (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents (scale 1 larm-force) :initial-element 0))
	  (setq half-wrench (scale 0.5 (v+ (send *ik-param* :get-rarm-wrench) (send *ik-param* :get-larm-wrench))))
	  (send *ik-param* :set-rleg-wrench   (v+ (send *ik-param* :get-rleg-wrench-offset) (scale -1 half-wrench)))
	  (send *ik-param* :set-lleg-wrench   (v+ (send *ik-param* :get-lleg-wrench-offset) (scale -1 half-wrench)))

          ;; (setq ret (ik-opt rarm-coords larm-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision?  :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :arm (if (equal target "board") arm "rarm")  :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15))
	  (setq ret (ik-opt rarm-coords larm-coords  :fix-contact? fix-contact?  :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift  :kin-task-value-thre 0.15 :eom-trans-task-value-thre 30 :heave? heave? :arm arm))
      (send *robot-env* :dissoc-root-virtual)))
    ;; (send *robot* :head-neck-p :joint-angle -40)
    (format t "ret ik = ~A~%" ret)
    (setq com (elt fc 1))
    ;; (format t "com =~A~%" com)

    ;;rviz visualize normal vector
    (progn
      (setq normal-start (instance geometry_msgs::PointStamped :init))
      (setq normal-end (instance geometry_msgs::PointStamped :init))
      (send normal-start :header :stamp (ros::time-now))
      (send normal-start :header :frame_id "odom")
      (send normal-start :point :x (* 0.001 (elt (v+ com #f(0 0 0)) 0))) (send normal-start :point :y (* 0.001 (elt (v+ com #f(0 0 0)) 1))) (send normal-start :point :z (* 0.001 (elt (v+ com #f(0 0 0)) 2)))
      (send normal-end :header :stamp (ros::time-now))
      (send normal-end :header :frame_id "odom")
      (send normal-end :point :x (* 0.001 (elt (v+ com (scale 200 normal-vec)) 0))) (send normal-end :point :y (* 0.001 (elt (v+ com (scale 200 normal-vec)) 1))) (send normal-end :point :z (* 0.001 (elt (v+ com (scale 200 normal-vec)) 2)))
      (ros::publish "/normal_start" normal-start)
      (ros::publish "/normal_end" normal-end)
      )

    ;; (progn (send (send *irtviewer* :viewer) :draw-arrow (v+ com #f(0 0 0)) (v+ com normal-vec) :color #f(0 0 1)) (send *irtviewer* :viewer :flush))
    ;; (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object? :wait? t)

    ;; ;; hand offset
    ;; (format t "hand offset")
    ;; (move-hand :larm (scale -1 (elt reaching-offset 0)) :send? nil)
    ;; (move-hand :rarm (scale -1 (elt reaching-offset 1)) :send? nil)
    (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
    (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
    (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? fix-contact? :look-at-object? look-at-object? :wait? (not check?) :fix-hand? fix-hand?)
    ;; (send *robot* :head-neck-p :joint-angle -40)
;; (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
    (if check?
        (progn
          ;; (setq lnum-thre (* 0.5 (send (one-shot-subscribe "/larm/num" std_msgs::Int32 :timeout 5000) :data))
          ;;       rnum-thre (* 0.5 (send (one-shot-subscribe "/rarm/num" std_msgs::Int32 :timeout 5000) :data)))
          (check-func :working? t )
          )
      )
    
    (setq lnum-after (send (one-shot-subscribe "/larm/num" std_msgs::Int32 :timeout 5000) :data)
          rnum-after (send (one-shot-subscribe "/rarm/num" std_msgs::Int32 :timeout 5000) :data))
    (format t "lnum-diff = ~A~%" (- lnum-pre lnum-after))
    (format t "rnum-diff = ~A~%" (- rnum-pre rnum-after))
    (if (< thre-lnum (abs (- lnum-pre lnum-after)))
        (move-hand :larm #f(300 0 0) :send? send?))
    (if (< thre-rnum (abs (- rnum-pre rnum-after)))
        (move-hand :rarm #f(300 0 0) :send? send?))
    ;; (setq ret-arm-save ret-arm)
    (setq normal-vec-save normal-vec)
    (setq l-unit-save l-unit)
    (setq r-unit-save r-unit)
    (format t "l coords fin ~A ~%" (send *robot* :larm :end-coords :worldpos))
    (format t "r coords fin ~A ~%" (send *robot* :rarm :end-coords :worldpos))
    ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
    (send *robot* :move-coords (send *coords* :get-lleg-coords) (send *robot* :lleg :end-coords))
    (format t "l-pos-diff=~A~%"    (v- (send *robot* :larm :end-coords :worldpos) (send l-act-coords-save :worldpos)))
    (setq l-pos-diff (normalize-vector (v- (send *robot* :larm :end-coords :worldpos) (send l-act-coords-save :worldpos))))
    (setq r-pos-diff (normalize-vector (v- (send *robot* :rarm :end-coords :worldpos) (send r-act-coords-save :worldpos))))
    (format t "~%l-pos-diff = ~A" l-pos-diff)
    (format t "r-pos-diff = ~A~%" r-pos-diff)
    (setq l-act-coords-save (send *robot* :larm :end-coords :copy-worldcoords))
    (setq r-act-coords-save (send *robot* :rarm :end-coords :copy-worldcoords))
    ;; (setq l-normal-vec-scaled-save l-normal-vec-scaled)
    ;; (setq r-normal-vec-scaled-save r-normal-vec-scaled)
    (format t "~%~%~%~%~%")
    ;; (setq ret-rarm-save ret-rarm)
    )
)

;;holding check function 0213
(defun check-func
    (&key (working? t)
          (send? t)
          (focus-arm "both")
          (num-thre-list (list (/ (send (one-shot-subscribe "/rarm/num" std_msgs::Int32 :timeout 5000) :data) 2) (/ (send (one-shot-subscribe "/larm/num" std_msgs::Int32 :timeout 5000) :data) 2)))
          (force-thre-list (list (* 0.8 (abs (elt (send *ik-param* :get-rarm-wrench) 2))) (* 0.8 (abs (elt (send *ik-param* :get-larm-wrench) 2))))))

  ;;片手のみに注目する際には反対の手の閾値を０にする
  (set-inhand-thre (cond ((equal focus-arm "both") num-thre-list) ((equal focus-arm "rarm") (list (elt num-thre-list 0) 0)) ((equal focus-arm "larm") (list 0 (elt num-thre-list 1)))))

  ;;片手のみに注目する際には反対の手の閾値を10000とする
  (setq force-thre-list (cond ((equal focus-arm "both") force-thre-list) ((equal focus-arm "rarm") (list (elt force-thre-list 0) 10000)) ((equal focus-arm "larm") (list 10000 (elt force-thre-list 1)))))
  
  (setq holding-flag (check-force-pcl :working? working? :thre-list force-thre-list))
  (setq pos-shift 230)
  (reset-coords)
  
  (if (equal holding-flag '(1 1))
      (progn
        (print "holding")
        (return-from check-func nil))
    (progn
      (send *ri* :stop-motion)
      (format t "~%stop~%~%")
      ;; (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift #f(0 0 0) :rarm-shift #f(0 0 0) :rot-z 0))
      (setq coords-list (auto-set-reaching-targets-each-hand :pos-shift pos-shift :larm-shift #f(0 0 0) :rarm-shift #f(0 0 0) :rot-z 0))
      (cond
       ((equal holding-flag '(0 1)) ;;rarmが外れている
        (setq rarm-coords (elt coords-list 1))
        (setq rarm-coords (send rarm-coords :rotate (deg2rad -20) :z))
        (move-hand :rarm #f(-150 0 0) :send? send?)
        (send *ri* :move-gripper :rarm 0 :wait t)
        (move-hand :rarm (v- (send rarm-coords :worldpos) (send *robot* :rarm :end-coords :worldpos)) :send? send? :frame :world)
        )

       ((equal holding-flag '(1 0)) ;;larmが外れている
        (setq larm-coords (elt coords-list 0))
        (setq larm-coords (send larm-coords :rotate (deg2rad -20) :z))
        (move-hand :larm #f(-150 0 0) :send? send?)
        (send *ri* :move-gripper :larm 0 :wait t)
        (move-hand :larm (v- (send larm-coords :worldpos) (send *robot* :larm :end-coords :worldpos)) :send? send? :frame :world))
 
       ((equal holding-flag '(0 0))
        (my-init-pose :tilt? nil :wait? t)
        (move-on-foot (send *robot* :rleg :end-coords :copy-worldcoords) (send *robot* :lleg :end-coords :copy-worldcoords) (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(-100 0 0) :world) (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(-100 0 0) :world))
        (reach-auto :send? send? :pos-shift 310 :adjust? nil :ik1? nil :larm-shift #f(0 -10 0) :rarm-shift #f(0 10 0) :fix-contact? 2 :reaching-offset (list #f(-50 -0 0) #f(-50 0 0)) :rot-z 5))
       )
      ))
  )
    
;;rerach targets 確認用
;; (defun test-reach-prepare()
(defun reach-auto-prepare()
  (my-init-pose :move-foot? nil)
  (start-coords)
  ;; (send_box_size :size-vector #f(3 1.5 2.0) :box-position #f(1 0 0))
  (send *ri* :start-impedance :arms)
  (send *ri* :set-reference-force-updater-param :arms :motion-dir #f(0 0 -1) :p-gain 0.01 :update-time-ratio 1.0)
  (send *ri* :start-reference-force-updater :arms)
  (send *ri* :set-auto-balancer-param :use-limb-stretch-avoidance t :limb-length-margin #f(0.06 0.06 0 0) :limb-stretch-avoidance-vlimit #f(-10e-5 1e-5))
  ;; (send *robot* :head-neck-p :joint-angle 23)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  )
;; (setq pre-angle (send *robot* :reset-manip-pose))
;; (setq pre-lleg (make-coords :pos #f(0 -100 0)))

;;首の調整込み
;;reach-auto :target "stick" :send? t :shift-stick #f(0 0 -200)
;;reach-auto :pos-shift 280 :reaching-offset (list #f(-140 50 0) #f(-140 50 0))  ;;rotate
(defun reach-auto
    (&key  (fix-contact? nil) (send? t) (target "board") (pos-shift 280) (one-hand? nil) (adjust? t) (ik1? t) (ik2? t) (ik3? nil) (only-move-foot? t) (larm-shift #f(0 -30 0)) (rarm-shift #f(0 40 0)) (pull? t)
           (kin-task-value-thre 0.01) (eom-rot-task-value-thre 0.01) (eom-rot-trans-value-thre 0.01)
            (larm-force #f(0 0 -40))
            (rarm-force #f(0 0 -40))
            (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
            (rleg-pos-shift #f(90 -30 0))
            (lleg-pos-shift #f(90 30 0))
            (look-at-object? t)
            (fix-hand? nil)
            (reaching-offset (list #f(-120 -0 0) #f(-120 0 0)))  ;;right left
            (arm "larm")
            (rot-z 20)
            (rot-x 0)
            (rot-y 0)
            (shift-stick #f(30 20 -40))
            (move-left? nil))

  (reset-coords)
  (if (equal target "board")
      (progn
        (setq obj (make_plane))
        (send *ik-param* :set-object (car obj))
        ;; (setq ret (get-target-pose obj))
        (setq ret (get-target-pose))
        (setq center-nearhands (elt ret 3))
        ;; (setq pos-centroid (elt ret 5))
        (setq longest-len (elt ret 4))
        (format t "longest-len = ~A~%" longest-len)
        (format t "pos =~A ~%" center-nearhands)
        (setq set-unique-kin-scale? nil)
        (if (or one-hand? (< longest-len 10))
            (progn 
              (setq pos-shift 0)
              (if (equal arm "larm")
                  (setq shift-standing-pos -50)
                (setq shift-standing-pos 50))
              ;; (setq arm "larm")
              )
          (progn 
            ;; (setq pos-shift 300)
            (print "both arms")
            (setq shift-standing-pos 0)
            (setq arm "both")))
        (if (and one-hand? move-left?)
            (progn 
              (setq shift-standing-pos 150)
              (setq axis 1)
              (setq edge-axis 0)
              (setq pos-shift -30)
              (setq arm "rarm"))
          (progn
            (setq axis 0)
            (setq edge-axis 1))
          )
        )
    ;;stick
    (progn
      (setq stick-coords (get-stick-pose :shift shift-stick))
      (setq center-nearhands (send stick-coords :worldpos))
      (setq shift-standing-pos 300)
      (if adjust?
          (send *ri* :go-pos (* 0.001 (- (elt center-nearhands 0) 600))  0 0))
      (setq set-unique-kin-scale? t)
      (setq axis 0)
      (setq edge-axis 1)
      (send *ik-param* :set-kin-scale-list (list
                         ;; 1.0
                         ;; 1.0
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 1 0.1)
                         0.0))
      ))
  (setq pos-shift 230) ;;remove-left-flat-onehand
  (format t "set-unique-kin-scale? =~A~%" set-unique-kin-scale?)
  (format t "center-nearhands =~A ~%" center-nearhands)
  
  (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))  

  ;; (if (equal arm "larm")
  ;;     (setq shift-standing-pos -50)
  ;;   (setq shift-standing-pos 50))
  (setq center-pos (scale 0.5 (v+ (send lleg-coords :worldpos) (send rleg-coords :worldpos))))

  ;;板の目の前に移動
  (if adjust?
      (progn
        (format t "~A~%" shift-standing-pos)
        (format t "(- (elt center-nearhands 1) (elt center-pos 1)) =~A~%" (* 0.5 (- (elt center-nearhands 1) (elt center-pos 1))))
        (setq rleg-coords-new (send (send rleg-coords :copy-worldcoords) :translate (float-vector 0 (+ shift-standing-pos (* 0.5 (- (elt center-nearhands 1) (elt center-pos 1)))) 0)))
        (setq lleg-coords-new (send (send lleg-coords :copy-worldcoords) :translate (float-vector 0 (+ shift-standing-pos (* 0.5 (- (elt center-nearhands 1) (elt center-pos 1)))) 0)))
        (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)
        (format t "lleg-coords = ~A~%" (send *robot* :lleg :end-coords :copy-worldcoords))
        (format t "rleg-coords = ~A~%" (send *robot* :rleg :end-coords :copy-worldcoords))
        
        ))
  (unix::sleep 2)

  (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents rarm-force :initial-element 0))
  (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents larm-force :initial-element 0))
  (setq half-wrench (scale 0.5 (v+ (send *ik-param* :get-rarm-wrench) (send *ik-param* :get-larm-wrench))))
  (send *ik-param* :set-rleg-wrench   (v+ (send *ik-param* :get-rleg-wrench-offset) half-wrench))
  (send *ik-param* :set-lleg-wrench   (v+ (send *ik-param* :get-lleg-wrench-offset) half-wrench))
  
  (reset-coords)
  (if ik1?
      (progn
        ;;1回目は板のエッジそのままの位置にリーチング出来る立ち位置を決める
        (if (equal target "board")
            ;; (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift larm-shift :rarm-shift rarm-shift :rot-z rot-z :axis axis :edge-axis edge-axis :rot-y rot-y :rot-x rot-x))
          (setq coords-list (auto-set-reaching-targets-each-hand :pos-shift (* 2 pos-shift) :larm-shift larm-shift :rarm-shift rarm-shift :rot-z rot-z :rot-y rot-y :rot-x rot-x))
          (setq coords-list (list stick-coords stick-coords #f(0 0 0))))

        ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
        (setq neck-angle (send *robot* :head-neck-p :joint-angle))

        ;;平面が見つからない時
        (while (and (not coords-list) (< neck-angle 20))
          (setq neck-angle (+ neck-angle 5))
          (if (equal target "board")
            ;; (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift larm-shift :rarm-shift rarm-shift :rot-z rot-z :axis axis :edge-axis edge-axis :rot-y rot-y :rot-x rot-x))
          (setq coords-list (auto-set-reaching-targets-each-hand :pos-shift (* 2 pos-shift) :larm-shift larm-shift :rarm-shift rarm-shift :rot-z rot-z :rot-y rot-y :rot-x rot-x))
            (progn
              (setq stick-coords (get-stick-pose :shift shift-stick))
              (setq coords-list (list stick-coords stick-coords #f(0 0 0)))))
          (format t "look down angle = ~A~%" neck-angle)
          (send *robot* :head-neck-p :joint-angle neck-angle)
          (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
          (send *ri* :wait-interpolation)
          )
        (setq center-pos-from-auto-set-reaching-targets (elt coords-list 2))
        
        (format t "neck-angle = ~A~%" neck-angle)
        (if (not coords-list)
            (return-from reach-auto nil))

        (send *ri* :start-log)
        
        ;; (setq center-pos (scale 0.5 (v+ (send larm-coords :worldpos) (send rarm-coords :worldpos))))
        ;; (format t "center-pos = ~A~%" center-pos)
        (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
        (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
        (send *coords* :set-rleg-coords rleg-coords)
        (send *coords* :set-lleg-coords lleg-coords)
        (format t "lleg-coords = ~A~%" lleg-coords)
        (format t "rleg-coords = ~A~%" rleg-coords)
        (setq lleg-pos-shift (v+ lleg-pos-shift (float-vector 0 0 0)))
        (setq rleg-pos-shift (v+ rleg-pos-shift (float-vector 0 0 0)))
        ;; (format t "rleg-pos-shift = ~A~%" rleg-pos-shift)
        ;; (setq fix-contact? nil)
        ;;手先力適当

        ;; (send *ik-param* :set-rarm-wrench  (make-array 6 :element-type float-vector :initial-contents rarm-force :initial-element 0))
        ;; (send *ik-param* :set-larm-wrench  (make-array 6 :element-type float-vector :initial-contents larm-force :initial-element 0))
        ;; (setq half-wrench (scale 0.5 (v+ (send *ik-param* :get-rarm-wrench) (send *ik-param* :get-larm-wrench))))
        ;; (send *ik-param* :set-rleg-wrench   (v+ (send *ik-param* :get-rleg-wrench-offset) half-wrench))
        ;; (send *ik-param* :set-lleg-wrench   (v+ (send *ik-param* :get-lleg-wrench-offset) half-wrench))

        (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
        (setq ret (ik-opt (elt coords-list 1) (elt coords-list 0) :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :arm (if (equal target "board") arm "rarm")  :set-unique-kin-scale? set-unique-kin-scale? :obst? nil))
        (send *robot-env* :dissoc-root-virtual)
        ;;上手く解けなかった場合
        (if (equal ret nil)
            (progn
              ;;足の領域を前に出してeom trans task の値を小さくする
              (reset-coords)
              (format t "change leg pos shift +30 ~%")
              (setq lleg-pos-shift (v+ lleg-pos-shift (float-vector 30 0 0)))
              (setq rleg-pos-shift (v+ rleg-pos-shift (float-vector 30 0 0)))
              (setq ret (ik-opt (elt coords-list 1) (elt coords-list 0) :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :arm (if (equal target "board") arm "rarm")  :set-unique-kin-scale? set-unique-kin-scale?))
              (send *robot-env* :dissoc-root-virtual)))

        ;; (format t "hand offset~%")
        ;; (setq angle-vec (send *robot* :angle-vector))
        ;; (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
        ;; (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
        ;; (move-hand :rarm (scale +1 (elt reaching-offset 0)) :send? nil)
        ;; (move-hand :larm (scale +1 (elt reaching-offset 1)) :send? nil)

        (setq center-pos-from-auto-set-reaching-targets (v- center-pos-from-auto-set-reaching-targets (send *robot* :lleg :end-coords :worldpos);; (midpoint 0.5 (send *robot* :rleg :end-coords :worldpos) (send *robot* :lleg :end-coords :worldpos))
                                                            ))
        ;;立ち位置変更
        (format t "change position~%")
        (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
        (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
        (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact? :only-move-foot? t :fix-hand? fix-hand? :look-at-object? look-at-object?)
        (unix::sleep 2)

        ))
  (setq fix-contact? 2)
  ;;IK 2 その場でターゲットに手を伸ばす
  (if ik2?
      (progn
        ;;2回目の脚の位置は目の状態から動かないため
        (reset-coords)
        (setq only-move-foot? nil)
        (setq rleg-pos-shift #f(0 0 0))
        (setq lleg-pos-shift #f(0 0 0))
        (setq rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
        (setq lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
        (setq waist-pos (send (car (send *robot* :links)) :worldpos))
        (if (equal target "board")
            ;; (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift larm-shift  :rarm-shift rarm-shift :rot-z rot-z :center-pos (if ik1? center-pos-from-auto-set-reaching-targets waist-pos) :axis axis :edge-axis edge-axis :rot-y rot-y :rot-x rot-x))
            (setq coords-list (auto-set-reaching-targets-each-hand :pos-shift (* 2 pos-shift) :larm-shift larm-shift  :rarm-shift rarm-shift :rot-z rot-z ;; :center-pos (if ik1? center-pos-from-auto-set-reaching-targets waist-pos) :axis axis :edge-axis edge-axis
                                                         :rot-y rot-y :rot-x rot-x))

          (progn
            (setq stick-coords (get-stick-pose :shift shift-stick))
            (setq coords-list (list stick-coords stick-coords))))
 
        ;;平面が見つからない時
        ;; (setq neck-angle (send *robot* :head-neck-p :joint-angle))
        ;; (while (and (not coords-list) (< -40 neck-angle ))
        ;;   (setq neck-angle (+ neck-angle 5))
        ;;   (if (equal target "board")
        ;;       (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift #f(0 0 0)  :rarm-shift #f(0 0 0) :rot-z rot-z))
        ;;     (progn
        ;;       (setq stick-coords (get-stick-pose))
        ;;       (setq coords-list (list stick-coords stick-coords))))
        ;;   (format t "look down angle = ~A~%" neck-angle)
        ;;   (send *robot* :head-neck-p :joint-angle neck-angle)
        ;;   (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
        ;;   (send *ri* :wait-interpolation)
        ;;   )
        ;; (objects (list *robot*))
        ;; (format t "neck-angle = ~A~%" neck-angle)
        (if (not coords-list)
            (return-from reach-auto nil))
        
        (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
        (setq ret (ik-opt (elt coords-list 1) (elt coords-list 0) :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :arm (if (equal target "board") arm "rarm")  :set-unique-kin-scale? set-unique-kin-scale? :reset? nil))
        (send *robot-env* :dissoc-root-virtual)

        ;;腕を引いた姿勢
        (format t "hand offset~%")
        (setq angle-vec (send *robot* :angle-vector))
        (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
        (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
	(if pull?
	    (progn
	      (move-hand :rarm (scale +1 (elt reaching-offset 0)) :send? nil)
          (if (equal target "board")         
              (move-hand :larm (scale +1 (elt reaching-offset 1)) :send? nil))
	      (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact? :fix-hand? fix-hand? :look-at-object? nil)
	))
        ;; (if send?
        ;;     (progn
        ;;       (send *robot* :head-neck-p :joint-angle 12)
        ;;       (send-pose-and-move-on-foot :time 6000 :send? send? :fix-contact? 2 :only-move-foot? nil :fix-hand? fix-hand? :look-at-object? nil)
        ;; (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
        ;; (unix::sleep 1)))
        ;;腕を戻す
        (setq fix-contact? 2)
        (send *robot* :angle-vector angle-vec)
        (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
        (format t "change pose~%")
        (send-pose-and-move-on-foot :time 8000 :send? send? :fix-contact? fix-contact?  :fix-hand? fix-hand? :look-at-object? look-at-object?)
        ))

  (if ik3?
      (progn
        (reset-coords)
        (format t "rleg-pos-shift = ~A~%" rleg-pos-shift)
        ;;IK 3  少し手前からターゲットへ手をのばす
        (if (equal target "board")
            (progn
              ;; (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift larm-shift :rarm-shift rarm-shift :rot-z rot-z :axis axis :edge-axis edge-axis :rot-y rot-y :rot-x rot-x))
              (setq coords-list (auto-set-reaching-targets-each-hand :pos-shift pos-shift :larm-shift larm-shift :rarm-shift rarm-shift :rot-z rot-z :rot-y rot-y :rot-x rot-x))              
              (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
              (setq ret (ik-opt  (elt coords-list 1) (elt coords-list 0) :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :arm (if (equal target "board") arm "rarm")  :set-unique-kin-scale? set-unique-kin-scale? :reset? t))
              (send *robot-env* :dissoc-root-virtual)
              (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact? :fix-hand? fix-hand? :look-at-object? look-at-object?))
          (progn  ;;stick
            (move-hand :larm l-diff :send? send?)
            (move-hand :rarm r-diff :send? send?)))))

  (send *ri* :move-gripper :arms 1 :wait t)
  ;; (send *ri* :move-gripper :larm 0.6 :wait t) (send *ri* :move-gripper :rarm 0.6 :wait t)
  )
  
;;1203
;;progn (load "demo.l ") (prepare-simulator)
;;reaching-pose :send? t
;;check-force-both-arms 
(defun check-force-both-arms
    (&key  (fix-contact? 2) (send? t) (cog #f(0 0 700))
               (wait? t) (reset? t) (apply-external-force? t) (avoid-collision? t)
               (rleg-pos-shift #f(0 0 0))
               (lleg-pos-shift #f(0 0 0))
               (look-at-object? nil)
               (thre -10.0))
  (let ((time-tmp 3000)
        (i 0))
  ;; (if (not (= 0 (check-pcl)))
  ;;     (progn
  ;;       (format t "adjust~%")
  ;;       (move-hand :larm #f(30 40 0) :time time-tmp)
  ;;       (move-hand :rarm #f(30 -40 0) :time time-tmp)))
  
  (setq larm-coords-tmp (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rarm-coords-tmp (send *robot* :rarm :end-coords :copy-worldcoords))
  (setq center-pos-tmp (scale 0.5 (v+ (send larm-coords-tmp :worldpos) (send rarm-coords-tmp :worldpos))))
  (setq larm-force (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
  (setq rarm-force (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
  (setq cnt 0)
  (send *ri* :start-log)
  ;;両手の高さを揃えたい
  (setq ave-hight (midpoint 0.5 (send larm-coords-tmp :worldpos) (send larm-coords-tmp :worldpos)))
  (setq adjust-hight (v+ ave-hight #f(10 0 30)))
  (move-hand :larm (v- (float-vector (elt (send larm-coords-tmp :worldpos) 0) (elt (send larm-coords-tmp :worldpos) 1) (elt adjust-hight 2)) (send larm-coords-tmp :worldpos)) :time time-tmp :frame :world)
  (move-hand :rarm (v- (float-vector  (elt (send rarm-coords-tmp :worldpos) 0) (elt (send rarm-coords-tmp :worldpos) 1) (elt adjust-hight 2)) (send rarm-coords-tmp :worldpos)) :time time-tmp :frame :world)
  (format t "larm adjust pos = ~A~%" (send *robot* :larm :end-coords :copy-worldcoords))
  (format t "rarm adjust pos = ~A~%" (send *robot* :rarm :end-coords :copy-worldcoords))
  (while (or (< thre (elt larm-force 2)) (< thre (elt rarm-force 2)))
    (format t  "cnt = ~A~%" cnt)
    (if (eq (mod (+ cnt 1) 5) 0)
        (progn
          (setq ave-hight (midpoint 0.5 (send *robot* :rarm :end-coords :worldpos) (send *robot* :rarm :end-coords :worldpos)))
          (setq adjust-hight (v+ ave-hight #f(10 0 30)))
          (move-hand :larm (v- (float-vector (elt (send *robot* :larm :end-coords :worldpos) 0) (elt (send *robot* :larm :end-coords :worldpos) 1) (elt adjust-hight 2)) (send *robot* :larm :end-coords :worldpos)) :time time-tmp :frame :world)
          (move-hand :rarm (v- (float-vector  (elt (send *robot* :rarm :end-coords :worldpos) 0) (elt (send *robot* :rarm :end-coords :worldpos) 1) (elt adjust-hight 2)) (send *robot* :rarm :end-coords :worldpos)) :time time-tmp :frame :world)
          (format t "larm adjust pos = ~A~%" (send *robot* :larm :end-coords :copy-worldcoords))
          (format t "rarm adjust pos = ~A~%" (send *robot* :rarm :end-coords :copy-worldcoords))))
    (if (eq (mod (+ cnt 1) 2) 0)
        (progn (print "larm")
               (move-hand :larm #f(5 40 0) :time time-tmp))
      (progn (print "rarm")
      (move-hand :rarm #f(5 -40 0) :time time-tmp)))
    (setq larm-force (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
    (setq rarm-force (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
    (setq cnt (+ cnt 1))
    (if (and (> thre (elt larm-force 2)) (> thre (elt rarm-force 2)))
        (return))
    )
  (format t "grasp~%")
  (setq total-force (v+ larm-force rarm-force))
  (setq lg (abs (elt larm-force 2)))
  (setq rg (abs (elt rarm-force 2)))
  (setq tg (abs (elt total-force 2)))
 
  (setq com-expected (v+ (scale (/ lg tg) (send larm-coords-tmp :worldpos)) (scale (/ rg tg) (send rarm-coords-tmp :worldpos))))
  (format t "larm-pos = ~A~%" (send larm-coords-tmp :worldpos))
  (format t "rarm-pos = ~A~%" (send rarm-coords-tmp :worldpos))
  (format t "center-pos = ~A~%" center-pos-tmp)
  (format t "com-expected = ~A~%" com-expected)
  (setq dif (v- com-expected center-pos-tmp))
  ;; (setq larm-target-coords (send larm-coords-tmp :translate dif :world))
  ;; (setq rarm-target-coords (send rarm-coords-tmp :translate dif :world))
  (format t "dif = ~A~%" dif)
  (format t "larm-target coords= ~A~%" larm-target-coords)
  (format t "rarm-target coords= ~A~%" rarm-target-coords)  
  (move-hand :larm (scale (- cnt 1) #f(-5 -30 0)) :time (+ time-tmp 2000))
  (move-hand :rarm (scale (- cnt 1) #f(-5  30 0)) :time (+ time-tmp 2000))
  ;; (format t "larm slide= ~A~%" (v- (send larm-target-coords :worldpos) (send larm-coords-tmp :worldpos)))
  ;; (format t "rarm slide= ~A~%" (v- (send rarm-target-coords :worldpos) (send rarm-coords-tmp :worldpos)))
  (move-hand :larm  dif  :frame :world)
  (move-hand :rarm dif :frame :world)
  ;; (setq lleg-pos-shift (v+ lleg-pos-shift (float-vector 0 (* 0.5 (elt dif 1)) 0)))
  ;; (setq rleg-pos-shift (v+ rleg-pos-shift (float-vector 0 (* 0.5 (elt dif 1)) 0)))
  ;; (ik-opt rarm-target-coords larm-target-coords :only-kinematics nil  :optimize-torque? nil :fix-contact? fix-contact? :avoid-collision? avoid-collision? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift )
  ;; (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (setq zmp-vector (send *ri*  :zmp-vector))
  ;; (setq rleg-coords-new (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (setq lleg-coords-new (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (setq lleg-ratio (abs (norm (v- (send lleg-coords-new :worldpos) zmp-vector))))
  ;; (setq rleg-ratio (abs (norm (v- (send rleg-coords-new :worldpos) zmp-vector))))
  ;; (if send?
  ;;     (progn 
  ;;       (if (not fix-contact?)
  ;;           (progn
  ;;             (send *ri* :set-gait-generator-param :zmp-weight-map (float-vector rleg-ratio lleg-ratio 1 1))
  ;;             (move-on-foot rleg-coords lleg-coords rleg-coords-new lleg-coords-new :wait? t)))
  ;;       (send-pose movetime :force-move flag-simulator :look-at-object? look-at-object?)))
  ))
;; send *ri* :set-gait-generator-param :default-step-time 1.2
;; (defun peer ()
;;   ;; (send *robot* :angle-vector #f(-8.63811 -17.6405 -97.7167 52.2116 4.78422 17.5815 8.11715 5.14641 -98.3516 55.5981 4.00441 -10.5335 0.0 0.0 0.0 0.0 30.0 0.0 55.0 -20.0 -15.0 -100.0 -25.0 0.0 -45.0 0.0 55.0 20.0 15.0 -100.0 25.0 0.0 -45.0))
;;   )

(defun predict-com-y(&key (time-tmp 5000))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  ;; (setq ave-hight (midpoint 0.5 (send larm-coords-tmp :worldpos) (send larm-coords-tmp :worldpos)))
  ;; (setq adjust-hight (v+ ave-hight #f(10 0 30)))
  ;; (move-hand :larm (v- (float-vector (elt (send larm-coords-tmp :worldpos) 0) (elt (send larm-coords-tmp :worldpos) 1) (elt adjust-hight 2)) (send larm-coords-tmp :worldpos)) :time time-tmp :frame :world)
  ;; (move-hand :rarm (v- (float-vector  (elt (send rarm-coords-tmp :worldpos) 0) (elt (send rarm-coords-tmp :worldpos) 1) (elt adjust-hight 2)) (send rarm-coords-tmp :worldpos)) :time time-tmp :frame :world)

  (send *ri* :start-log)
  (setq larm-force (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
  (setq rarm-force (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
  (setq larm-coords-init (send *robot* :larm :end-coords :copy-worldcoords))
  (setq rarm-coords-init (send *robot* :rarm :end-coords :copy-worldcoords))
  ;; (send *ri* :stop-impedance :arms)
  (send *ri* :remove-force-sensor-offset-rmfo-arms)
  (send *ri* :start-impedance :arms :m-p 100 :d-p 10000 :k-p 100 :k-r 200)
  (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq lf-list (list)
	rf-list (list)
	lp-list (list)
	rp-list (list)
	lpos-list (list 0)
	f-diff-list (list))
  (dotimes (i 5)
    ;; (send *ri* :move-gripper :arms 1 :wait t)
    (format t "hand up ~%")
    ;; (heave-with-sensor 50 50 1 :first-lift 50)
    (move-hand :larm #f(0 140 0) :send? t :wait? t)
    (unix::sleep 1)
    (setq larm-force-tmp (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
    (unix::sleep 1)
    (move-hand :larm #f(0 -120 0) :send? t :wait? t)
    (format t "hand down ~%")

    (format t "hand up ~%")
    (move-hand :rarm #f(0 -140 0) :send? t :wait? t)
    (unix::sleep 1)
    (setq rarm-force-tmp (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
    (unix::sleep 1)
    (move-hand :rarm #f(0 120 0) :send? t :wait? t)
    (format t "hand down ~%")
    
    ;; (move-hand :larm #f(0 0 50) :send? t :frame :world)
    ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
    (setq larm-coords-tmp (send *robot* :larm :end-coords :copy-worldcoords))
    (setq rarm-coords-tmp (send *robot* :rarm :end-coords :copy-worldcoords))
    (setq diff (midpoint 0.5 (v- (send larm-coords-tmp :worldpos) (send larm-coords-init :worldpos)) (v- (send rarm-coords-tmp :worldpos) (send rarm-coords-init :worldpos))))
    ;; (setq larm-force-tmp (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
    ;; (setq rarm-force-tmp (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
    (setq lf-list (append (list larm-force-tmp) lf-list)
          rf-list (append (list rarm-force-tmp) rf-list)
          lp-list (append (list larm-coords-tmp) lp-list)
          rp-list (append (list rarm-coords-tmp) rp-list)
          f-diff-list (append (list (- (elt larm-force-tmp 2) (elt  rarm-force-tmp 2))) f-diff-list))
    (format t "force diff = ~A~%" (elt f-diff-list i))
    ;; (setq move-dir (normalize-vector (float-vector (elt (send rarm-coords-tmp :z-axis) 0) (elt (send rarm-coords-tmp :z-axis) 1) 0)))
    (setq move-dir #f(0 1 0))
    (if (< (abs (elt larm-force-tmp 2)) (abs (elt rarm-force-tmp 2)))
        (progn
	  (format t "move left~%")
          (setq lpos-list (append  (list (+ (elt lpos-list i)  +1)) lpos-list))
          ;; (setq lpos (+ lpos -1))
          )
      (progn
	(format t "move right~%")
        (setq lpos-list (append  (list (+ (elt lpos-list i) -1)) lpos-list))
	(setq move-dir (scale -1 move-dir))))

    (setq walk-dist (scale 100 move-dir))

    
    (send *ri* :move-gripper :arms 0.1 :wait t)
    
    (format t "walk direction ~A ~%" move-dir)
    (format t "walk distance ~A ~%" walk-dist)
    
    (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
    (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
    (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
    (send *coords* :set-rleg-new-coords (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate walk-dist :world))
    (send *coords* :set-lleg-new-coords (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate walk-dist :world))
    (send-pose-and-move-on-foot :time 5000 :send? t :fix-contact? nil :look-at-object? t :wait? t :fix-hand? nil :only-move-foot? t)
    )
  (format t "f-diff-list = ~A~%" f-diff-list)
  )
  
  
  
(defun init-steps-pose(&key (send? nil) (fix-contact? t))
  (send *robot* :angle-vector #f(-0.768 -11.3775 -19.404 44.316 -24.85 11.4007 0.78 11.13 -20.304 44.712 -24.36 -11.1503 -0.0075 0.0 0.0 0.0 5.00625 1.53125 64.325 -20.5625 -16.5187 -104.687 -28.5562 -1.58125 -49.0932 -1.51875 64.375 20.45 15.9687 -105.056 28.3125 2.175 -49.0932))
  (send *robot* :move-coords (make-coords :pos #f(0 280 0)) (send *robot* :lleg :end-coords))
  (objects (list *robot* ))
  ;; (send *robot* :head-neck-p :joint-angle -40)  
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  )

;;持ち上げてみて面積を比べる
(defun serch-top-board()
  (move-hand :larm  #f(30 200 0) :wait? t)
  (unix::sleep 2)
  (setq l-fc (make_plane :set-com? nil))
  (send *ik-param* :set-object (car l-fc))
  (setq l-area (send (car (send (elt l-fc 0) :get-face :prism :top)) :area))
  (setq l-com (elt l-fc 1))
  (move-hand :larm  #f(-20 -200 0) :wait? t)

  (move-hand :rarm  #f(30 -200 0) :wait? t)
  (unix::sleep 2)
  (setq r-fc (make_plane :set-com? nil))
  (send *ik-param* :set-object (car r-fc))
  (setq r-area (send (car (send (elt r-fc 0) :get-face :prism :top)) :area))
  (setq r-com (elt r-fc 1))
  (move-hand :rarm  #f(-20 200 0) :wait? t)

  (if (< l-area r-area)
      (set_plane_centroid l-com)
    (set_plane_centroid r-com)
    )
  )

(defun predict-k()
  (setq row 6)
  (setq dir-list (list #f(1 0 0) #f(-1 0 0) #f(0 1 0) #f(0 -1 0) #f(0 0 1) #f(0 0 -1) ))
  (setq scl 80)
  (setq wrench-mat (make-matrix row 6))
  (setq pose-mat (make-matrix row 6))
  (dotimes (i row)
    (progn (setq r-f (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
	   (setq r-m (low-pass-online "moment" "off" :rarm :times 10 :sleep 100 :flag nil))
	   (setq vec (float-vector (elt r-f 0) (elt r-f 1) (elt r-f 2) (elt r-m 0) (elt r-m 1) (elt r-m 2)))
	   (format t "vec=~A~%" vec)
	   (setq r-coords (send *robot* :rarm :end-coords :copy-worldcoords))
	   (format t "dir =~A~% " (elt dir-list i))
	   (move-hand :rarm (scale scl (elt dir-list i)))
	   (send *robot* :angle-vector (send *ri* :state :angle-vector))
	   (setq r-coords-new (send *robot* :rarm :end-coords :copy-worldcoords))
	   (setq r-f-new (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
	   (setq r-m-new (low-pass-online "moment" "off" :rarm :times 10 :sleep 100 :flag nil))
	   (setq vec-new (float-vector (elt r-f-new 0) (elt r-f-new 1) (elt r-f-new 2) (elt r-m-new 0) (elt r-m-new 1) (elt r-m-new 2)))
	   (setq dif-vec (v- vec-new vec))
	   (format t "dif-vec=~A~%" dif-vec)
	   (setq dif-coords (send r-coords :transformation r-coords-new :world))
	   (setq dif-pos (send dif-coords :worldpos))
	   (setq dif-rpy (car (rpy-angle  (send dif-coords :worldrot))))
	   
	   (progn (setf (aref pose-mat i 0) (elt dif-pos 0)) (setf (aref pose-mat i 1) (elt dif-pos 1)) (setf (aref pose-mat i 2) (elt dif-pos 2)) (setf (aref pose-mat i 3) (elt dif-rpy 0)) (setf (aref pose-mat i 4) (elt dif-rpy 1)) (setf (aref pose-mat i 5) (elt dif-rpy 2)))
	   (dotimes (j 6) (setf (aref wrench-mat i j) (elt dif-vec j))) (format t "reset ~%") (move-hand :rarm (scale (* -1.0 scl) (elt dir-list i)))
	   )
    )
  (list wrench-mat pose-mat)
  (setq pose-mat-inv (transpose pose-mat))
  (setq wrench-mat-inv (transpose wrench-mat))
  (setq k-mat (m* wrench-mat-inv (pseudo-inverse pose-mat-inv)))
  
  )

(defun predict-k-dim3
    (&key (dir-list (list #f(0 0 1) #f(0 0 -1) #f(1 0 0) #f(-1 0 0) #f(0 1 0) #f(0 -1 0))) (force-scl 10) (pose-scl 30) (move-by-force? nil) (use-force? t) (send? nil) (frame :local) (movetime-tmp 3000))
  (setq row (length dir-list))
  (setq dim 3)
  (setq wrench-mat (make-matrix row dim))
  (setq pose-mat (make-matrix row dim))
  ;; (setq move-ref #f(0 0 40))
  ;; (setq weight-vector (low-pass-online "force" "off" :rarm :times 100 :sleep 10000 :flag nil))
  ;; (format t "move =~A~% " move-ref)
  ;; (move-hand :rarm move-ref :time 5000 :frame frame)
  ;; (setq move-ref #f(0 0 -30))
  ;; (format t "move =~A~% " move-ref)
  ;; (move-hand :rarm move-ref :time 5000 :frame frame)
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :move-coords (make-coords :pos #f(0 100 0)) (send *robot* :lleg :end-coords :copy-worldcoords))
  (setq pre-pose (send *robot* :angle-vector))
  (dotimes (i row)
    (progn
      (send *robot* :angle-vector (send *ri* :state :angle-vector))
      (send *robot* :move-coords (make-coords :pos #f(0 100 0)) (send *robot* :lleg :end-coords :copy-worldcoords))
      (objects (list *robot*))
      (unix::usleep 500000)
      ;; (setq pre-pose (send *robot* :angle-vector))
      (setq r-coords (send *robot* :rarm :end-coords :copy-worldcoords))
      (send r-coords :draw-on :flush nil :size 300 :color #f(0 0 1))
      (format t "r-coords *ri*=~A~%" r-coords)
      (send *ri* :remove-force-sensor-offset-rmfo-arms)
      (unix::sleep 1)
      (print "rmfo")
      (if move-by-force?
          (progn 
            (send *ri* :set-ref-force (scale force-scl (elt dir-list i)) 2000 :rarm)
            (format t "dir =~A~% " (elt dir-list i))
            (print "set ref force")            
            )
        (progn
          (setq move-ref (scale pose-scl (elt dir-list i)))
          (format t "move =~A~% " move-ref)
          (move-hand :rarm move-ref :time movetime-tmp :frame frame)
          ;;ik opt version
          ;; (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
          ;; (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
          ;; (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
          ;; (setq ret (ik-opt (send (send r-coords :copy-worldcoords) :translate move-ref :world) (send *robot* :larm :end-coords :copy-worldcoords) :arm "rarm" :only-kinematics nil  :optimize-torque? nil :fix-contact? 2 :avoid-collision? nil :apply-external-force? nil :wait? t  :set-unique-kin-scale? t :kin-scale-list (list 1 1 1 0) :kin-task-value-thre 0.15 :eom-rot-task-value-thre 10) )
          ;; (send *robot-env* :dissoc-root-virtual)
          ;; (send-pose-and-move-on-foot :send? send? :fix-contact? 2 :fix-hand? nil :time 4000)
          (format t "move =~A~% " move-ref)
          ;; (format t "r-coords move-hand *robot* =~A~%" (send *robot* :rarm :end-coords :copy-worldcoords))
          (objects (list *robot*))
          (unix::sleep 1)))

      (setq r-f (low-pass-online "force" "off" :rarm :times 100 :sleep 100000 :flag nil))

      (send *robot* :angle-vector (send *ri* :state :angle-vector))
      (send *robot* :move-coords (make-coords :pos #f(0 100 0)) (send *robot* :lleg :end-coords :copy-worldcoords))
      (objects (list *robot*))
      (unix::usleep 500000)
      (setq r-coords-new (send *robot* :rarm :end-coords :copy-worldcoords))
      (send r-coords-new :draw-on :flush t :size 300 :color #f(0 1 0))
      (format t "r-coords-new *ri*=~A~%" r-coords-new)
      ;; (setq dif-coords (send r-coords :transformation r-coords-new :world))
      (setq dif-pos (v- (send r-coords-new :worldpos) (send r-coords :worldpos)))
      ;; (format t "dif-coords=~A~%" dif-coords)
      ;; (setq dif-pos (send dif-coords :worldpos))
      (format t "dif-pos=~A~%" dif-pos)
      ;; (setq dif-rpy (car (rpy-angle  (send dif-coords :worldrot))))
      
      (progn (setf (aref pose-mat i 0) (elt dif-pos 0)) (setf (aref pose-mat i 1) (elt dif-pos 1)) (setf (aref pose-mat i 2) (elt dif-pos 2)))
      (if use-force?
          (dotimes (j dim)
            (setf (aref wrench-mat i j) (elt r-f j))))
      (format t "reset pose  ~%")
      (send *robot* :angle-vector pre-pose)
      (send *robot* :move-coords (make-coords :pos #f(0 100 0)) (send *robot* :lleg :end-coords))
      (if move-by-force?
          (send *ri* :set-ref-force #f(0 0 0) 2000 :rarm)
        (progn
          ;; (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
          ;; (setq ret (ik-opt (send r-coords-new :translate (scale -1 move-ref) :world) (send *robot* :larm :end-coords :copy-worldcoords) :arm "rarm" :only-kinematics nil  :optimize-torque? nil :fix-contact? 2 :avoid-collision? nil :apply-external-force? nil :wait? t  :set-unique-kin-scale? nil :kin-task-value-thre 0.15 :eom-rot-task-value-thre 10))
          ;; (send *robot-env* :dissoc-root-virtual)
          ;; (send-pose-and-move-on-foot :send? send? :fix-contact? 2 :fix-hand? nil :time 4000)))
          (move-hand :rarm (scale -1 move-ref) :time movetime-tmp :frame frame)
          ))
      ;; (send *ri* :angle-vector (send *robot* :angle-vector) movetime-tmp)
      (send-pose-and-move-on-foot :send? send? :fix-contact? t :fix-hand? nil :time movetime-tmp :wait? t)
      (objects (list *robot*))
      (unix::sleep 1)
      )
    )
  (setq pose-mat-T (transpose pose-mat))
  (if use-force?
      (progn
        (setq wrench-mat-T (transpose wrench-mat))
        (setq k-mat (m* wrench-mat-T (pseudo-inverse pose-mat-T)))
        (setq movable-vector (pred-k :mat k-mat :wrench-mat wrench-mat-T :pose-mat pose-mat-T :use-force? use-force?)))
    (progn
      (setq movable-vector (pred-k :mat k-mat :pose-mat pose-mat-T :use-force? use-force?))))
  ;; (setq min-vector (elt (elt  eign-eignvector-list 1) 2))
  movable-vector
  )

;;my-init-pose :send? t
;;move-hand :rarm #f(300 80 -150) :send? t
;;pull-out 50
(defun pull-out(&key (pull-scl 60) (pull-out-dir #f(0 0 0.5)) (send? t) (dir-list (list #f(0 0 1) #f(0 0 -1) #f(1 0 0) #f(-1 0 0) #f(0 1 0) #f(0 -1 0))))
  (setq i 0)
  (setq move-ref #f(0 0 40))
  (format t "move =~A~% " move-ref)
  (move-hand :rarm move-ref :time 4000 :frame :local)
  (setq weight-vector (low-pass-online "force" "off" :rarm :times 100 :sleep 10000 :flag nil))
  (setq move-ref #f(0 0 -30))
  (format t "move =~A~% " move-ref)
  (move-hand :rarm move-ref :time 4000 :frame :local)
  
  ;; (do-until-key
  (while t
    (format t "iteration ~A~%" i)
    (if (< 0 i)
	(setq dir-list (append (list movable-vector) dir-list)))
    (setq movable-vector (predict-k-dim3 :use-force? nil :send? send? :dir-list dir-list))
    (format t "~%~%pull scale ~A~%" pull-scl)
    (setq move-pos (scale pull-scl (normalize-vector (v+ movable-vector pull-out-dir))))
    (format t "move ~A~%~%" move-pos)
    (move-hand :rarm  move-pos :wait? t)
    (setq i (+ i 1)))
  )
  
(defun reaching-pose-high(&key (send? t) (fix-contact? nil))
  ;;両足裏が水平になってない
    ;; (send *coords* :set-rleg-new-coords (make-coords :pos #f(100.497 -176.101 -0.009) :rpy #f(-0.024 -2.481e-07 1.708e-06)))
  ;; (send *coords* :set-lleg-new-coords (make-coords :pos #f(104.745 109.342 -0.009) :rpy #f( -0.007 -3.460e-07 -1.236e-06)))
  ;; (send *coords* :set-rleg-new-coords (make-coords :pos #f(100.497 -176.101 -0.009) :rpy #f(-0.0 0 0)))
  ;; (send *coords* :set-lleg-new-coords (make-coords :pos #f(104.745 109.342 -0.009) :rpy #f( -0.00 0 0)))
  ;; (send *robot* :angle-vector #f(0.897233 -5.17316 -30.3655 70.966 -32.5286 3.39279 0.817664 2.15006 -29.2349 69.2021 -31.8935 -3.86937 -0.084729 13.3282 0.165979 0.113276 26.0 37.0 23.572 -24.405 5.82802 -61.7925 45.8089 -15.8428 9.01074 -37.0 22.7147 22.8779 -5.17243 -54.4999 -36.9803 22.1565 19.5136))

  ;;足が前後に開いている
  ;; (setq lcoords (send (make-coords :pos #f(1.641e-13 -3.808e-14 1.705e-13) :rpy #f( 4.615e-18 -5.575e-17 2.776e-17)) :translate #f(60 170 0)))
  ;; (send *coords* :set-lleg-new-coords lcoords)
  ;; (format t "lcoords =~A~%" lcoords)
  ;; (setq rcoords (send (make-coords :pos #f(-128.208 -349.071 -0.03) :rpy #f(0.052 2.726e-06 4.098e-05)) :translate #f(60 170 0)))
  ;; (send *coords* :set-rleg-new-coords rcoords)
  ;; (format t "rcoords =~A~%" rcoords)
  ;; (send *robot* :angle-vector #f(0.576351 -8.58841 -49.3331 72.3813 -32.2978 8.38789 -0.281562 4.89258 -59.1932 67.249 -17.2386 -4.78654 6.0 23.3898 -12.4538 2.93443 -5.0 37.0 20.361 -38.6374 -12.9775 -68.3973 42.3067 -22.3984 9.44434 -37.0 9.53446 26.8474 5.81806 -77.1031 -58.4279 13.6935 25.763))

  ;; (setq lcoords (make-coords :pos #f(2.832e-14 -3.453e-14 4.263e-14) :rpy #f(-2.075e-17 1.663e-18 2.776e-17)))
  ;; (send lcoords :translate #f(8 180 0))
  ;; (setq rcoords (make-coords :pos #f(16.216 -370.186 -0.002) :rpy #f(-0.041 -7.191e-07 5.665e-06)))
  ;; (send rcoords  :translate #f(8 180 0))
  ;; (send *robot* :angle-vector  #f(-1.76859 -6.82574 -46.7277 85.9558 -39.4514 6.83213 0.648952 9.15973 -44.2373 85.3872 -41.3745 -9.1629 -4.58023 21.1258 1.08981 -1.38348 -5.0 37.0 16.9453 -29.5958 -4.01948 -73.3314 48.134 -13.0157 20.4156 -37.0 21.3996 33.0076 6.06592 -71.559 -44.4047 15.882 17.5909))
  ;; (setq lcoords (make-coords :pos #f(1.092e-17 -1.293e-14 0.0) :rpy #f(3.465e-18 -1.820e-19 -1.388e-17)))
  ;; (send lcoords :translate #f(30 160 0))
  ;; (setq rcoords (make-coords :pos #f(-59.735 -321.362 -0.007) :rpy #f(0.021 9.093e-07 1.857e-06)))
  ;; (send rcoords :translate #f(30 160 0))
  ;; (send *robot* :angle-vector  #f(0.845168 -4.76947 -39.7486 76.4653 -40.7972 4.69775 0.428295 5.99958 -45.3741 76.0349 -34.7505 -6.01463 -1.82797 7.89905 -5.87246 4.67899 1.83123 37.0 10.28 -27.6137 -0.426646 -64.7098 50.3253 -4.42444 15.1117 -37.0 23.5686 28.2448 6.40036 -65.7202 -40.0836 13.8851 15.4743))
  ;;0113
  ;; (llegcoords (make-coords :pos #f(-1.137e-13 1.915e-14 1.705e-13) :rpy #f(-3.466e-18 1.546e-19 1.388e-17)))
  ;; 	(rlegcoords (make-coords :pos #f( -29.787 -318.721 0.002) :rpy #f(-0.029 -1.529e-06 -4.615e-06)))
    ;; (send llegcoords :translate #f(15 160 0) :world)
  ;; (send rlegcoords :translate #f(15 160 0) :world)
  
  (let ((llegcoords (make-coords :pos #f(15.0 160.0 1.705e-13) :rpy #f(-3.466e-18 1.546e-19 1.388e-17)))
	(rlegcoords (make-coords :pos #f(-14.787 -158.721 0.002) :rpy #f(-0.029 -1.529e-06 -4.615e-06))))
    (reset-coords)
    (send *robot* :angle-vector  #f(-1.704 -6.44234 -42.038 88.5287 -47.1159 6.46033 0.05511 5.10885 -45.2001 88.979 -44.4028 -5.10899 1.13229 11.6047 -7.23663 2.69917 0.824779 37.0 26.4666 -34.0665 -4.80595 -73.4077 43.7561 -8.55578 13.5856 -37.0 25.2936 19.536 4.02469 -77.5973 -48.1484 12.0398 25.3355))
  
  
    (send *coords* :set-lleg-new-coords llegcoords)
    (send *coords* :set-rleg-new-coords rlegcoords)
    (format t "lcoords =~A~%" llegcoords)
    (format t "rcoords =~A~%" rlegcoords)
    (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
    ;; (send *robot* :move-coords (send *coords* :get-rleg-new-coords) (send *robot* :rleg :end-coords))
    ;; (if fix-contact?
    ;; 	(send *robot* :move-coords (send (send *coords* :get-rleg-new-coords) :translate #f(-198 0 0)) (send *robot* :rleg :end-coords))
  ))

(defun board-holding-pose-2(&key (send? t) (fix-contact? nil))
  (send *coords* :set-rleg-new-coords (make-coords :pos #f(50 -249 -0.009) :rpy #f(-0.024 -2.481e-07 1.708e-06)))
  (send *coords* :set-lleg-new-coords (make-coords :pos #f(48.356 245.005 -0.008) :rpy #f(0.022 4.965e-06 1.208e-05)))
  (send *robot* :angle-vector #f(0.198761 -11.1901 -15.2719 36.987 -13.1589 10.9564 -0.398351 10.8076 -15.1034 36.9009 -13.2446 -10.8891 0.0 3.0 -0.026228 -0.005043 18.4018 37.0 38.4206 -88.7584 -30.2222 -80.624 -3.43998 -3.26935 26.4778 -37.0 39.2722 90.1039 30.227 -79.7785 4.40195 2.48036 26.5397))
  (send *ri* :start-log)
  (send *robot* :move-coords (send *coords* :get-rleg-new-coords) (send *robot* :rleg :end-coords))
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  )

(defun emergency-check-pose(&key (send? t) (fix-contact? nil))
  ;; (send *coords* :set-rleg-new-coords (make-coords :pos #f(156.984 1.812 -0.008) :rpy #f(-0.009 6.967e-07 -3.684e-06)))
  ;; (send *coords* :set-lleg-new-coords (make-coords :pos #f(156.985 261.812 -0.012) :rpy #f(0.014 1.315e-06 -9.876e-07)))
  ;;   (send *coords* :set-rleg-new-coords (make-coords :pos #f(156.984 -130 -0.008) :rpy #f(-0.009 6.967e-07 -3.684e-06)))
  ;; (send *coords* :set-lleg-new-coords (make-coords :pos #f(156.985 132 -0.012) :rpy #f(0.014 1.315e-06 -9.876e-07)))
  ;;足首ゆるくて転倒したので横に開くようにした一時的に
  (send *coords* :set-rleg-new-coords (make-coords :pos #f(0.984 -134 -0.008) :rpy #f(-0.009 6.967e-07 -3.684e-06)))
  (send *coords* :set-lleg-new-coords (make-coords :pos #f(0.985 132 -0.012) :rpy #f(0.014 1.315e-06 -9.876e-07)))  
  (send *robot* :angle-vector #f(-4.30965 2.78768 -46.61 64.8649 -19.4957 -0.259879 -2.8812 7.6575 -48.3618 65.4802 -18.4271 -5.15996 -1.76532 16.1927 -0.218483 -1.86632 10.0 37.0 -10.5421 -37.7312 11.7252 -48.9479 32.3991 -4.75814 18.1441 -37.0 -4.73756 49.4529 -5.47873 -48.3535 -34.726 -0.928479 3.70811))
  (send *ri* :start-log)
  (send *robot* :move-coords (send *coords* :get-rleg-new-coords) (send *robot* :rleg :end-coords))
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?))

;;力センサのログをとるため
(defun save-hand-force-log()
  (progn (send *ri* :start-log ) (unix::sleep 30) (save-log :log-dir "matsuura/log")))

  
(defun pred-multi-com
    (force1 moment1 force2 moment2 pre-coords post-coords &key (g  9.8))

  ;;make f-matrix

  ;;pre-coods座標系で絶対座標系で表現されたforce１を表現し直す
  (format t "coord1     = ~A~%" pre-coords)
  (format t "f1 (world) = ~A~%" force1)
  (setq f  (send pre-coords :inverse-rotate-vector force1))
  (setq m1 (send pre-coords :inverse-rotate-vector moment1))
  (format t "f1(coord1) = ~A~%" f)
  ;; (print (format nil "f1 = ~A" f))
  (setq f-matrix1 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (format t "f-matrix1 = ~A~%~%" f-matrix1)

  ;; (setq f (transform change-rot force2))
  ;;小島さんより
  ;;post-coods座標系で,絶対座標系で表現されたforce2を表現し直す
  (format t "coord2     = ~A~%" post-coords)
  (format t "f2 (world) = ~A~%" force2)
  ;; (setq f2  (send post-coords :inverse-rotate-vector force2))
  ;; (setq mm2 (send post-coords :inverse-rotate-vector moment2))
  (setq f  (send post-coords :inverse-rotate-vector force2))
  (setq m2 (send post-coords :inverse-rotate-vector moment2))
  (format t "f2(coord2) = ~A~%" f)
  ;; (print (format nil "f2 = ~A" f2))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる
  ;; (setq f  (send (send post-coords :transformation pre-coords :world) :inverse-rotate-vector f2))
  ;; (setq f  (send (send post-coords :transformation pre-coords) :inverse-rotate-vector f2))
  ;; (setq m2 (send (send post-coords :transformation pre-coords :world) :inverse-rotate-vector mm2))
  (format t "f2(coord1) = ~A~%" f)
  (setq f-matrix2 (matrix (float-vector 0 (- 0 (elt f 2)) (elt f 1)) (float-vector (+ 0 (elt f 2)) 0 (- 0 (elt f 0)))  (float-vector (- 0 (elt f 1)) (elt f 0) 0)))
  (format t "f-matrix2 = ~A~%~%" f-matrix2)

  (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
  (print (format nil "moment  = ~A" m))

  ;;f-matrixをくっつける
  (setq f-matrix (matrix (matrix-row f-matrix1 0) (matrix-row f-matrix1 1)  (matrix-row f-matrix1 2) (matrix-row f-matrix2 0) (matrix-row f-matrix2 1)  (matrix-row f-matrix2 2)))
  (print (format nil "f-matrix = ~A" f-matrix))

  ;;momentベクトルをくっつける
  ;;絶対座標系で表現されたmoment１をpre-coods座標系で表現し直す
  ;; (setq m1 (send pre-coords :inverse-rotate-vector moment1))
  ;;小島さんより
  ;; (setq m2 (transform change-rot moment2))
  ;;絶対座標系で表現されたmoment2をpost-coods座標系で表現し直す
  ;; (setq mm2 (send post-coords :inverse-rotate-vector moment2))
  ;;pre-coodsに座標系が重なるようにベクトルを回転させる  
  ;; (setq m2 (send (send post-coords :transformation pre-coords :world) :rotate-vector mm2))
  ;; (setq m (float-vector (elt m1 0) (elt m1 1)  (elt m1 2) (elt m2 0) (elt m2 1)  (elt m2 2)))
  ;; (print (format nil "moment  = ~A" m))

  (setq f-inverse-matrix (pseudo-inverse f-matrix))
  (format t "f-inverse-matrix = ~A~%" f-inverse-matrix)

  (setq com (scale -1 (transform f-inverse-matrix m)))
  (format t "com=~A with pseudo-inverse~%" com)
  (format t "com norm = ~A~%" (norm com))
  (setq com-world (send pre-coords :rotate-vector com))
  (format t "com-world =~A with pseudo-inverse~%" com-world)
  (format t "com-local norm = ~A~%" (norm com-world))
  com
)

(defun pred-com(&key (send? t) (fix-contact? t) (hand-offset #f(100 0 0)) (arm :rarm))
  (send *robot* :reset-pose)
  (send *robot* arm :elbow-p :joint-angle -80)
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  (unix::sleep 2)
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  
  (setq force-vector (low-pass-online "force" "off" arm :times 100 :sleep 10000 :flag nil))
  (setq moment-vector (low-pass-online "moment" "off" arm :times 100 :sleep 10000 :flag nil))
  ;; (setq force-vector (low-pass-online "force" "off" arm :times 100 :sleep 10000 :flag nil))
  ;; (setq moment-vector (low-pass-online "moment" "off" arm :times 100 :sleep 10000 :flag nil))
  ;; (setq coords (send *robot* arm :end-coords :copy-worldcoords))
  (setq coords (send (send *robot* arm :end-coords :copy-worldcoords) :translate (scale -1 hand-offset) :local)) ;;sensor position
  (send *robot* arm :elbow-p :joint-angle -60)
  (send-pose-and-move-on-foot :send? send? :fix-contact? fix-contact?)
  (unix::sleep 2)
  (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (setq force-vector-next (low-pass-online "force" "off" arm :times 100 :sleep 10000 :flag nil))
  (setq moment-vector-next (low-pass-online "moment" "off" arm :times 100 :sleep 10000 :flag nil))
  ;; (setq force-vector-next (low-pass-online "force" "off" arm :times 100 :sleep 10000 :flag nil))
  ;; (setq moment-vector-next (low-pass-online "moment" "off" arm :times 100 :sleep 10000 :flag nil))

  (setq coords-next (send (send *robot* arm :end-coords :copy-worldcoords) :translate (scale -1 hand-offset) :local)) ;;sensor position

  (setq com-local-vector (pred-multi-com force-vector moment-vector force-vector-next moment-vector-next coords coords-next))
  (setq mass-world-vector (scale 0.5 (v+ force-vector force-vector-next)))
  ;; (setq com-local-vector-sub-offset (v- com-local-vector #f(0.10 0 0)))
  ;; (format t "com-local-vector subtract  offset=~A~%" com-local-vector-sub-offset )
  ;; (format t "com-local-vector subtract  offset norm =~A~%" (norm com-local-vector-sub-offset) )
  (setq com-local-vector-clear-offset (v+ com-local-vector (scale 0.001 hand-offset)))
  (format t "com-local-vector clear  offset=~A~%" com-local-vector-clear-offset )
  (format t "com-local-vector clear  offset norm =~A~%" (norm com-local-vector-clear-offset) )
  (list com-local-vector mass-world-vector com-local-vector-clear-offset)
  )

(defun rhand-calib(times)
  (setq flag-simulator t)
  (setq movetime 4000)
  (setq com-list (list #f(0 0 0)))
  (setq mass-list (list #f(0 0 0)))
  (setq sum-com #f(0 0 0))
  (setq sum-mass #f(0 0 0))
  (dotimes (i times)
    (format t "~% iteration ~A~%" i)
    (setq ret (pred-com))
    (setq com-list (append  (list (elt ret 0)) com-list))
    (setq mass-list (append  (list (elt ret 1)) mass-list))
    (setq sum-com (v+ sum-com (elt ret 0)))
    (setq sum-mass (v+ sum-mass (elt ret 1))))
  (format t "com-list =~A~%" com-list)
  (format t "mass-list =~A~%" mass-list)
  (setq ave-com (scale (/ 1.0 times) sum-com))
  (setq ave-mass (scale (/ 1.0 times) sum-mass))
  (setq flag-simulator nil)
  (setq movetime 12000)
  (list ave-com ave-mass))


(defun predict-collision-point(com-local-vector mass-world-vector)
  (setq coords (send *robot* arm :end-coords :copy-worldcoords))
  (setq z-axis (send (send *robot* arm :end-coords :copy-worldcoords) :axis :z))
  (setq force-vector (low-pass-online "force" "off" arm :times 100 :sleep 10000 :flag nil))
  (setq force (v- force-vector mass-world-vector))
  (setq moment-vector (low-pass-online "moment" "off" arm :times 100 :sleep 10000 :flag nil))
  (setq moment (v- moment-vector (v* force-vector (send coords :rotate-vector com-local-vector))))
  (setq scl-candidates (list (/ (elt moment 0) (elt force 0)) (/ (elt moment 1) (elt force 1)) (/ (elt moment 2) (elt force 2))))
  (setq scl (/ (+ (elt scl-candidates 0) (elt scl-candidates 1) (elt scl-candidates 2)) 3.0))
  (format t "scl-candidates =~A~%" scl-candidates)
  (format t "scl =~A~%" scl)
  scl)

(defun change-arm-auto
    (&key (fix-contact? nil)
          (send? nil)
          (wait? t)
          (avoid-collision? nil)
          (apply-external-force? nil)
          ;; (lleg-pos-shift #f(50 0 0))
          ;; (rleg-pos-shift #f(50 0 0))
          (rleg-pos-shift #f(80 -20 0))
          (lleg-pos-shift #f(80 20 0))
          (look-at-object? t)
          (arm "larm")
          (fix-hand?)
          (holding-arm-coords (send *robot* :rarm :end-coords))
          )
  (setq movetime-tmp 5000)
  (setq contact-wall (make_plane))
  (setq face-arm (get-half-plane (elt contact-wall 0) :cut-rate 0.34))
  (format t "face-arm =~A~%" face-arm)
  (send *ik-param* :set-object face-arm)
  (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (ik-opt-change-arm  holding-arm-coords :arm arm  :fix-contact? fix-contact? :apply-external-force? apply-external-force? :wait? wait? :rleg-pos-shift rleg-pos-shift :lleg-pos-shift lleg-pos-shift :face-arm face-arm)
  (send *robot-env* :dissoc-root-virtual)

  (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (send-pose-and-move-on-foot :time movetime-tmp :send? send? :fix-contact? fix-contact? :look-at-object? look-at-object? :wait? wait? :fix-hand? fix-hand?)
  )

(defun reset-coords()
  (format t "~%~%reset lleg coords ")
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  (send *robot* :move-coords (make-coords) (send *robot* :lleg :end-coords))
  (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (format t "~%robot lcoords = ~A~%" (send *robot* :lleg :end-coords :copy-worldcoords))
  (format t "robot rcoords = ~A~%" (send *robot* :rleg :end-coords :copy-worldcoords))
  ;; (format t "ri    lcoords = ~A~%" (send *robot* :lleg :end-coords :copy-worldcoords))
  ;; (format t "ri    rcoords = ~A~%" (send *robot* :rleg :end-coords :copy-worldcoords))
  (format t "robot angle-vector = ~A~%" (send *robot* :angle-vector))
  (objects (list *robot*))
  )

(defun change-arm-auto-without-plane
    (&key (fix-contact? nil)
          (send? nil)
          (wait? t)
          (avoid-collision? nil)
          (apply-external-force? nil)
          ;; (lleg-pos-shift #f(50 0 0))
          ;; (rleg-pos-shift #f(50 0 0))
          (rleg-pos-shift #f(80 -20 0))
          (lleg-pos-shift #f(80 20 0))
          (look-at-object? t)
          (arm "larm")
          (fix-hand?)
          (holding-arm-coords (send *robot* :rarm :end-coords))
	  (rot-z 20)
	  (pos-shift 280)
	  (only-hand? nil)
	  (force-move nil))  ;; (let* ((coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift #f(200 -150 0) :rarm-shift #f(200 150 0) :rot-z rot-z)))
  ;; (send *ri* :start-log)
  (reset-coords)
  (send *ik-param* :set-force-move t)
  (setq movetime-tmp 3000)
  (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  ;; (send *robot* :angle-vector (send *ri* :state :angle-vector))
  ;; (reset-coords)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  ;; (move-hand :arms #f(20 0 0) :send? t :rotation-axis? t)
  (send *ri* :set-auto-balancer-param :default-zmp-offsets (list #f(0 20 0.0) #f(0 -20 0.0) #f(0.0 0.0 0.0) #f(0.0 0.0 0.0)))
  (unix::sleep 1)
  ;; (setq coords-list (auto-set-reaching-targets :pos-shift pos-shift :larm-shift #f(200 -150 0) :rarm-shift #f(200 150 0) :rot-z rot-z))
  ;; (setq rarm-coords (elt coords-list 1))
  (if only-hand?
      (progn
        (send *ri* :move-gripper :rarm 0 :wait t)
        (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
        (move-hand :rarm #f(-120 0 0) :send? t :rotation-axis? t :time movetime-tmp)
        (send *ri* :move-gripper :rarm 1 :wait t)
        (move-hand :rarm #f(-80 0 -240) :send? t :frame :world :rotation-axis? t :time movetime-tmp)
        ;; (move-hand :rarm #f(-60 200 0) :send? t)
        (send rarm-coords :translate #f(0 0 -160) :world)
        (move-hand :rarm (v- (send rarm-coords :worldpos) (send *robot* :rarm :end-coords :worldpos)) :send? t :frame :world :rotation-axis? t :time movetime-tmp :rotation-axis? nil) 

        (send *ri* :move-gripper :larm 0 :wait t)
        ;; (setq larm-coords (elt coords-list 0))
        (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
        (move-hand :larm #f(-160 0 0) :send? t :rotation-axis? t :time movetime-tmp)
        (send *ri* :move-gripper :larm 1 :wait t)
        (move-hand :larm #f(-80 0 -240) :send? t :frame :world :rotation-axis? t :time movetime-tmp)
        ;; (move-hand :larm #f(-60 -200 0) :send? t)
        (send larm-coords :translate #f(10 0 -160) :world) 
        ;; (setq larm-coords (send larm-coords :translate #f(100 -150 0))) 
        (move-hand :larm (v- (send larm-coords :worldpos) (send *robot* :larm :end-coords :worldpos)) :send? t :frame :world :rotation-axis? t :time movetime-tmp :rotation-axis? nil)
        )
    
    (progn
      (send *ri* :move-gripper :rarm 0 :wait t)
      (setq rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
      (setq ret (ik-opt (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(-120 0 0) :local) (send *robot* :larm :end-coords :copy-worldcoords)  :fix-contact? 2  :apply-external-force? t :wait? t   :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15 :reset? t))
      (send *robot-env* :dissoc-root-virtual)
      (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
      (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
      (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? 2 :look-at-object? t :wait? (not check?))
      (if check?
          (check-func :working? t :focus-arm "larm"))
      (send *ri* :move-gripper :rarm 1 :wait t)
      
      (setq ret (ik-opt (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate #f(-110 0 -240) :world) (send *robot* :larm :end-coords :copy-worldcoords)  :fix-contact? 2  :apply-external-force? t :wait? t   :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15 :reset? t))
      (send *robot-env* :dissoc-root-virtual)
      (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
      (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
      (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? 2 :look-at-object? t :wait? (not check?))
      (if check?
          (check-func :working? t :focus-arm "larm"))
      
      (setq ret (ik-opt (send (send (send rarm-coords :translate #f(70 50 -220) :world) :copy-worldcoords) :rotate (deg2rad -12) :z :local) (send *robot* :larm :end-coords :copy-worldcoords)  :fix-contact? 2  :apply-external-force? t :wait? t   :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15 :reset? t))
      (send *robot-env* :dissoc-root-virtual)
      (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
      (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
      (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? 2 :look-at-object? t :wait? (not check?))
      (if check?
          (check-func :working? t :focus-arm "larm"))


      (send *ri* :move-gripper :larm 0 :wait t)
      (setq larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
      (setq ret (ik-opt (send *robot* :rarm :end-coords :copy-worldcoords) (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(-160 0 0) :local)  :fix-contact? 2  :apply-external-force? t :wait? t   :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15))
      (send *robot-env* :dissoc-root-virtual)
      (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
      (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
      (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? 2 :look-at-object? t :wait? (not check?))
      (if check?
          (check-func :working? t :focus-arm "rarm"))
      (send *ri* :move-gripper :larm 1 :wait t)
      
      (setq ret (ik-opt (send *robot* :rarm :end-coords :copy-worldcoords) (send (send *robot* :larm :end-coords :copy-worldcoords) :translate #f(-120 0 -240) :world)  :fix-contact? 2  :apply-external-force? t :wait? t   :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15 :reset? t))
      (send *robot-env* :dissoc-root-virtual)
      (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
      (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
      (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? 2 :look-at-object? t :wait? (not check?))
      (if check?
          (check-func :working? t :focus-arm "rarm"))
      
      (setq ret (ik-opt (send *robot* :rarm :end-coords :copy-worldcoords) (send (send (send larm-coords :translate #f(70 -30 -200) :world) :copy-worldcoords) :rotate (deg2rad 12) :z :local)  :fix-contact? 2  :apply-external-force? t :wait? t   :kin-task-value-thre 0.15 :eom-trans-task-value-thre 0.15 :reset? t))
      (send *robot-env* :dissoc-root-virtual)
      (send *coords* :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
      (send *coords* :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
      (send-pose-and-move-on-foot :time 5000 :send? send? :fix-contact? 2 :look-at-object? t :wait? (not check?))
      (if check?
          (check-func :working? t :focus-arm "rarm"))      
      )
    )
  )

(print "load functions.l")

;; larm-coords next= #<coordinates #X7f6176c42a68  411.214 272.38 881.195 / 0.481 0.99 1.748>
;; rarm-coords next= #<coordinates #X7f6176c42d08  434.272 -412.57 894.724 / 0.196 0.999 -1.4
;; larm-coords next= #<coordinates #X7f61a4627550  514.705 289.401 1043.785 / 0.616 1.174 1.858>
;; rarm-coords next= #<coordinates #X7f61a4612508  533.323 -400.6 1062.898 / 0.301 1.184 -1.382>

