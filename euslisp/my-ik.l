
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/validate-jacobian.l")
;; (require "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require "~/catkin_ws/jaxon_tutorial/src/jsk_control/eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(print "load my-ik.l")

(defun my-ik
    (rarm-coords larm-coords &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1)))

  (if (eq rleg nil)
      (setq rleg  (send *coords* :get-rleg))
    (format t "rleg ~A" rleg))
  (if (eq lleg nil)
      (setq lleg  (send *coords* :get-lleg)))
  (if (eq cog nil)
      (setq cog  (send *coords* :get-cog)))
        
  (send *robot* :fullbody-inverse-kinematics
        (list rleg
              lleg
              rarm-coords
              larm-coords)
          
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
        :root-link-virtual-joint-weight virtual-joint-weight
        ;; :root-link-virtual-joint-weight #f(0.5 0.5 0.5 0.0 0.0 0.0)
        
        :additional-weight-list (list (list (send *robot* :torso :waist-y :child-link) 0.0)
                                      (list (send *robot* :torso :waist-p :child-link) 0.0)
                                      (list (send *robot* :torso :waist-r :child-link) 0.0)
                                      )
        
        ;; :translation-axis (list t t t)
        ;; :rotation-axis (list t t t)
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        :target-centroid-pos (v+ (midpoint 0.5 (send rleg :worldpos) (send lleg :worldpos))
                                 cog)
        :debug-view nil
        ;; :revert-if-fail t
        ;; :cog-translation-axis move-root-link
        :cog-translation-axis move-root-link
        )
  
  (send rarm-coords :draw-on :flush nil :size 300)
  (send larm-coords :draw-on :flush t   :size 300)
  )

(defun setup-jaxon_red-ik-opt
    (&key (only-kinematics? nil))
  ;; (setq *robot* (instance hrp2jsknts-robot :init))
  (setq *robot* (instance jaxon_red-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "RLEG_LINK5")
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "LLEG_LINK5")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (setq r-co (send (send *robot* :rarm :end-coords :copy-worldcoords) :rotate (deg2rad 90) :x))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_LINK7")
                        :coords (send (send (send *robot* :link "RARM_LINK7") :copy-worldcoords)
                              :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ;; r-co
                        ))
  (setq l-co (send (send *robot* :larm :end-coords :copy-worldcoords) :rotate (deg2rad -90) :x))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_LINK7")
                        :coords (send (send (send *robot* :link "LARM_LINK7") :copy-worldcoords)
                              :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345)))
                        ;; l-co
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless only-kinematics?

    (create-support-polygon *robot* (send *robot* :link "RARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "LARM_LINK7"))    
    (create-support-polygon *robot* (send *robot* :link "RLEG_LINK5"))
    (create-support-polygon *robot* (send *robot* :link "LLEG_LINK5"))
    )
  ;; disable min-max table
  ;; (dolist (jnt (send *robot* :joint-list))
  ;;   (send jnt :set-val 'joint-min-max-table nil)
  ;;   (send jnt :set-val 'joint-min-max-target nil)
  ;;   )
  ;;set narrow limit of min/max angle instead of min-max table
  ;; (send (send *robot* :joint "RLEG_JOINT0") :min-angle -35)
  ;; (send (send *robot* :joint "LLEG_JOINT0") :max-angle 35)
  ;; (send (send *robot* :joint "RLEG_JOINT2") :min-angle -70)
  ;; (send (send *robot* :joint "LLEG_JOINT2") :min-angle -70)
  ;; (send (send *robot* :joint "HEAD_JOINT1") :max-angle 18)

  (send (send *robot* :joint "HEAD_JOINT0") :max-angle 34)
  (send (send *robot* :joint "CHEST_JOINT0") :min-angle -0.5)
  (send (send *robot* :joint "CHEST_JOINT0") :max-angle 0.5)
  
  (send (send *robot* :joint "CHEST_JOINT1") :min-angle 0)
  ;; (send (send *robot* :joint "CHEST_JOINT1") :max-angle 30)
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 20)

  (send (send *robot* :joint "CHEST_JOINT2") :min-angle -35)
  (send (send *robot* :joint "CHEST_JOINT2") :max-angle 35)
    
  (send (send *robot* :joint "RLEG_JOINT0") :min-angle -16)
  (send (send *robot* :joint "LLEG_JOINT0") :max-angle 16)
  ;;0116
  (send (send *robot* :joint "RLEG_JOINT1") :min-angle -15)
  (send (send *robot* :joint "RLEG_JOINT1") :max-angle 15)
  (send (send *robot* :joint "LLEG_JOINT1") :min-angle -15)
  (send (send *robot* :joint "LLEG_JOINT1") :max-angle 15)
  
  ;; (send (send *robot* :joint "RLEG_JOINT3") :min-angle 64)
  ;; (send (send *robot* :joint "LLEG_JOINT3") :min-angle 64)
  ;; (send (send *robot* :joint "RLEG_JOINT3") :min-angle 70)
  ;; (send (send *robot* :joint "LLEG_JOINT3") :min-angle 70)

  ;;for move-left one hand
  (send (send *robot* :joint "RLEG_JOINT3") :min-angle 90)
  (send (send *robot* :joint "LLEG_JOINT3") :min-angle 90)
  

  (send (send *robot* :joint "RARM_JOINT0") :min-angle 0)
  (send (send *robot* :joint "RARM_JOINT0") :max-angle 40)
  (send (send *robot* :joint "LARM_JOINT0") :min-angle -40)
  (send (send *robot* :joint "LARM_JOINT0") :max-angle 0)
  
  ;;肩の特異点問題dem-step-simulator
  ;; (send (send *robot* :joint "RARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "RARM_JOINT1") :max-angle 80)
  ;; (send (send *robot* :joint "LARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "LARM_JOINT1") :max-angle 80)

  ;; (send (send *robot* :joint "RARM_JOINT1") :min-angle -140)
  ;; (send (send *robot* :joint "RARM_JOINT1") :max-angle 140)
  ;; (send (send *robot* :joint "LARM_JOINT1") :min-angle -140)
  ;; (send (send *robot* :joint "LARM_JOINT1") :max-angle 140)

  ;; (send (send *robot* :joint "RARM_JOINT1") :min-angle -100)
  (send (send *robot* :joint "RARM_JOINT1") :min-angle -90)
  (send (send *robot* :joint "RARM_JOINT1") :max-angle 70)
  ;; (send (send *robot* :joint "LARM_JOINT1") :min-angle -100)
  (send (send *robot* :joint "LARM_JOINT1") :min-angle -90)
  (send (send *robot* :joint "LARM_JOINT1") :max-angle 70)
;;腕が180度回転するのを防ぐ
  ;; (send (send *robot* :joint "LARM_JOINT3") :min-angle -90)
  ;; (send (send *robot* :joint "LARM_JOINT3") :max-angle 90)
  ;; (send (send *robot* :joint "RARM_JOINT3") :min-angle -90)
  ;; (send (send *robot* :joint "RARM_JOINT3") :max-angle 90)
;;0116
  (send (send *robot* :joint "LARM_JOINT2") :max-angle 90)
  (send (send *robot* :joint "RARM_JOINT2") :min-angle -90)
  
  (send (send *robot* :joint "LARM_JOINT3") :min-angle -70)
  (send (send *robot* :joint "LARM_JOINT3") :max-angle 70)
  (send (send *robot* :joint "RARM_JOINT3") :min-angle -70)
  (send (send *robot* :joint "RARM_JOINT3") :max-angle 70)

  (send (send *robot* :joint "LARM_JOINT5") :min-angle -70)
  (send (send *robot* :joint "LARM_JOINT5") :max-angle 70)
  (send (send *robot* :joint "RARM_JOINT5") :min-angle -70)
  (send (send *robot* :joint "RARM_JOINT5") :max-angle 70)

  (send (send *robot* :joint "RARM_JOINT7") :min-angle -60)
  ;; (send (send *robot* :joint "LARM_JOINT7") :min-angle -60)
  )

;;reachとかに使ってるやつ
(defun ik-opt
    (rarm-coords
     larm-coords 
     &key (arm "both")
     (heave? nil)
     (obst? t)
     (optimize-torque? (send *ik-param* :get-optimize-torque?))
     (avoid-collision? (send *ik-param* :get-avoid-collision?))
     (apply-external-force? (send *ik-param* :get-apply-external-force?))
     (fix-contact? (send *ik-param* :get-fix-contact?))
     (set-unique-kin-scale? (send *ik-param* :get-set-unique-kin-scale?))
     (kin-scale-list ;; (list 1.0
                     ;;        1.0
                     ;;        (list 1 1 1 1 1 0.2)
      ;;        0.0)
      (send *ik-param* :get-kin-scale-list))
     (only-kinematics? (send *ik-param* :get-only-kinematics?))
     (pre-process-func)
     (kin-task-value-thre (send *ik-param* :get-kin-task-value-thre))
     (eom-rot-task-value-thre (send *ik-param* :get-eom-rot-task-value-thre))
     (eom-trans-task-value-thre (send *ik-param* :get-eom-trans-task-value-thre))
     (convergence-check-func #'(lambda(sqp)
                                 (if (and (< (norm (send (send sqp :config-task) :kinematics-task-value :update? nil)) kin-task-value-thre)
                                          (< (norm (send (send sqp :config-task) :eom-rot-task-value :update? nil)) eom-rot-task-value-thre)
                                          (< (norm (send (send sqp :config-task) :eom-trans-task-value :update? nil)) eom-trans-task-value-thre)
                                          )
                                     t ;converge
                                   nil)))
     (reset? (send *ik-param* :get-reset?))
     (external-wrench-list   (list  (send *ik-param* :get-rleg-wrench)
                                   (send *ik-param* :get-lleg-wrench)
                                   (send *ik-param* :get-rarm-wrench)
                                   (send *ik-param* :get-larm-wrench)))
     (external-coords-list (send *ik-param* :get-external-coords-list))
     (rleg-pos-shift (send *ik-param* :get-rleg-pos-shift))
     (lleg-pos-shift (send *ik-param* :get-lleg-pos-shift))
     &allow-other-keys
     )
  (let ((inner-width (* 0.45 (abs (- (elt (send *robot* :lleg :end-coords :worldpos) 1) (elt (send *robot* :rleg :end-coords :worldpos) 1))))))
    (format t "~%~%~%arm    ~A~%~%~%" arm)
  (format t "external wrench-list=~A~%" external-wrench-list)
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (send *coords* :set-larm-coords (send *robot* :larm :end-coords :copy-worldcoords))

  (setq rleg-pos-shift (v+ rleg-pos-shift (float-vector 0 (* -1 (- (* 0.5 (+ inner-width 250)) inner-width)) -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift (float-vector 0 (- (* 0.5 (+ inner-width 250)) inner-width) -10)))
  (format t "rleg-pos-shift in ik = ~A~%" rleg-pos-shift)
  ;; (setq *ground-rleg* (make-cube 800 400 20))
  ;; (setq *ground-lleg* (make-cube 800 400 20))
  ;; (setq *ground-rleg* (make-cube 600 400 20))
  ;; (setq *ground-lleg* (make-cube 600 400 20))

  (setq *ground-rleg* (make-cube 500 (+ 250 inner-width) 20)) ;;200
  (setq *ground-lleg* (make-cube 500 (+ 250 inner-width) 20)) ;;200
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list 0 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list 0 0 0))
        )
  ;; (format t "ground-rleg in ik = ~A~%" (send *ground-rleg* :copy-worldcoords))
  (setq *env-list* (list *ground-rleg* *ground-lleg*))
  (send-all *env-list* :worldcoords)
  ;; (send-all *env-list* :set-color (float-vector 0.5 0.5 0.25))
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (setq *rarm-coords* (instance arrow-object :init))
  (send *rarm-coords* :newcoords
        ;; (make-coords :pos (float-vector 290 -400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad -90)))
        rarm-coords
        )

  (setq *larm-coords* (instance arrow-object :init))
  (send *larm-coords* :newcoords
        ;; (make-coords :pos (float-vector 290  400 320) :rpy (float-vector 0 (deg2rad  30) (deg2rad  90)))
        larm-coords
        )
  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    (setq *obst* (make-cube 80 1000 500))
    ;; (send *obst* :translate (float-vector (+ (/ (+ (elt (send rarm-coords :worldpos) 0) (elt (send larm-coords :worldpos) 0)) 2) 50) 0 100))
    (if heave?
	(send *obst* :translate (v+ (scale 0.5 (v+ (send rarm-coords :worldpos) (send larm-coords :worldpos))) #f(-30 0 -400)))
      (send *obst* :translate (v+ (scale 0.5 (v+ (send rarm-coords :worldpos) (send larm-coords :worldpos))) #f(0 0 -400))))
    ;; (send *obst* :set-color (float-vector 1 0 0) 0.5)
        (send *obst* :set-color (float-vector 0 1 1) 0.5)
    )
  ;; setup robot
  (setup-jaxon_red-ik-opt :only-kinematics? only-kinematics? )
  ;;fix-contact nil 足を自由な位置へ
  ;;t contact-coordsの位置へ
  ;;2 その場で
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*
			 :root-virtual-joint-min-angle-list (list -1.000000e+07 -1.000000e+07 -1.000000e+07 -1.745329e-08 -1.745329e+08 -1.745329e-08)
			 :root-virtual-joint-max-angle-list (list 1.000000e+07 1.000000e+07 1.000000e+07 1.745329e-08 1.745329e+08 1.745329e-08)))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "rleg")
                     (instance 2d-planar-contact :init :name "lleg")
                     ;; (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
	       (instance robot-environment :init :robot *robot*
			 :root-virtual-joint-min-angle-list (list -1.000000e+07 -1.000000e+07 -1.000000e+07 -1.745329e-08 -1.745329e+08 -1.745329e-08)
			 :root-virtual-joint-max-angle-list (list 1.000000e+07 1.000000e+07 1.000000e+07 1.745329e-08 1.745329e+08 1.745329e-08) :contact-list *contact-list*)
               ;; (instance robot-environment :init :robot *robot* :contact-list *contact-list*)
	       )
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         ;; (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose))
  ;; (send *robot* :torso-waist-p :joint-angle 30)
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  ;;04 make picture for thesis
   (progn (setq bxr (make-cube 200 200 250 )) (setq bxl (make-cube 200 200 250 )) (send bxr :set-color #f(0 0 1 0.5)) (send bxl :set-color #f(0 0 1 0.5))  (send bxr :newcoords (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords))) (send bxl :newcoords (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords))) )
  ;; (objects (append (list *robot* *rarm-coords* *larm-coords*  ;;bxr bxl;; (send *ik-param* :get-object) 
  ;;                        ) *env-list*  (if avoid-collision? (list *obst*))
  ;; 			   ))
  ;; (objects (append (list *robot* *rarm-coords* *larm-coords*  (if avoid-collision? *obst*)) *env-list*)
  ;;                  )
  ;; (objects  (append (list (send *ik-param* :get-object) (if avoid-collision? *obst*)) *env-list*))
  (objects (append (list  *robot*  ) )
                   )
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if (and (not only-kinematics?) optimize-torque?)
                      (send *robot-env* :drive-joint-list) nil)
                  :only-kinematics? only-kinematics?
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :robot :joint-list) :joint-angle)
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト
                  (cond ((eq fix-contact? t)
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)                               
                               ;; (send *ground-larm* :copy-worldcoords)
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)
                               ;; (send *robot-env* :contact "larm" :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *rarm-coords* :copy-worldcoords)
                               (send *larm-coords* :copy-worldcoords)
                               ;; (send *robot-env* :contact "larm" :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        ;; (send *robot-env* :robot :get :rarm-contact-coords)
                        ;; (send *robot-env* :robot :get :larm-contact-coords)
                        (send *robot-env* :robot  :rarm :end-coords)
                        (send *robot-env* :robot  :larm :end-coords)          
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   ;; (send *ground-larm* :copy-worldcoords)
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   ;; (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            ;; (send *robot-env* :robot :get :larm-contact-coords)
                            ))
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       ))
                  :kin-scale-list
                  (if set-unique-kin-scale?
                      kin-scale-list ;;足平は前を向くように
                      (cond
                       ((equal arm "both")
                        (list
                         ;; 1.0
                         ;; 1.0
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 1 1)
                         ;; (list 1 1 1 1 1 0.5)
                         ;; (list 1 1 1 1 1 0.5)
                         1.0
                         1.0))
                       ((equal arm "rarm")
                        (list
                         ;; 1.0
                         ;; 1.0
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 0.5 1)
                         0.0))
                       ((equal arm "larm")
                        (list
                         ;; 1.0
                         ;; 1.0
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 1 1)
                         0.0
                         (list 1 1 1 1 0.5 1)))))
                            ;;axis y周りは自由 heaveの時
                            ;; (list 1 1 1 1 0 1)
                            ;; (list 1 1 1 1 0 1)
                  :collision-pair-list
                  (if avoid-collision?
                      ;; (list 
                       ;; (flatten (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
		      (append (if obst? (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
			      ;; (mapcar #'(lambda (l) (list l (send *robot* :link "CHEST_LINK1"))) (flatten (send *robot-env* :robot :larm :links)))
			      ;; (mapcar #'(lambda (l) (list l (send *robot* :link "HEAD_LINK1"))) (flatten (send *robot-env* :robot :arms :links)))
			       (list (list (send *robot* :link "LLEG_LINK2") (send *robot* :link "LARM_LINK7")))
			       ;; (list (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "LARM_LINK3")))
			       ;; (list (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "LARM_LINK5")))
			       (list (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "LARM_LINK6")))
			       (list (list (send *robot* :link "LLEG_LINK3") (send *robot* :link "LARM_LINK6")))
                       ;; (list (send *robot* :link "LLEG_LINK1") (send *robot* :link "LARM_LINK6"))
                       (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "RARM_LINK7")))
		       (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "LLEG_LINK2")))
		       
                       ;; (list (send *robot* :link "RLEG_LINK1") (send *robot* :link "RARM_LINK6"))
                       ;; (list (send *robot* :link "LLEG_LINK1") (send *robot* :link "LARM_LINK7"))
                       ;; (list (send *robot* :link "RLEG_LINK1") (send *robot* :link "RARM_LINK7"))
                       )
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin 0.05
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;;多分絶対座標で与える
                        ;; (list (float-vector 0 -80 0 0 0 0))
                        ;; (list (float-vector 0 80 0 0 0 0))
                        ;; 力の正負が微妙かも
                        ;; (list (float-vector 0 0 -80 0 0 0)
                        ;;       (float-vector 0 0 -80 0 0 0))
                        external-wrench-list
                        ;;pivot
                        ;; (list (float-vector 0 -50 0 0 0 0))
                        ;; ;;support
                        ;; ;; (list (float-vector 0 0 -50 0 0 0))
                        ;; (list (float-vector -40 0 0 0 0 0))
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;; external-coords-list
                        (list
                         (send *robot-env* :robot :get :rleg-contact-coords)
                         (send *robot-env* :robot :get :lleg-contact-coords)
                         (send *robot-env* :robot :get :rarm-contact-coords)
                         (send *robot-env* :robot :get :larm-contact-coords))
                        
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords)
                        ;;       (send *robot-env* :robot :get :larm-contact-coords))
                        ;;pivot
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords))
                        ;;support
                        ;; (list (send *robot-env* :robot :link "LARM_LINK5"))
                        )
                    )
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  :no-print? nil
                  :convergence-check-func

                  ;; #'(lambda(sqp)
                  ;;     (if (and (< (norm (send (send sqp :config-task) :kinematics-task-value :update? nil)) 0.03)
                  ;;              (< (norm (send (send sqp :config-task) :eom-rot-task-value :update? nil)) 0.2))
                  ;;         t ;converge
                  ;;       nil))
                  convergence-check-func
                  :failure-callback-func
                  #'(lambda(val)
                      (print "fail"))
                  ))
  (send *sqp-opt* :optimize :loop-num 200)
  (send (send *robot* :joint "RLEG_JOINT3") :min-angle 0)
  (send (send *robot* :joint "LLEG_JOINT3") :min-angle 0)
  (if (equal (send *sqp-opt* :status) :not-converged)
      (progn
        (format t "ik failed~%")
        (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
        (send *robot* :move-coords (send *coords* :get-lleg-coords) (send *robot* :lleg :end-coords))
        nil
        )
    t)
  )
  ;; (send *sqp-opt* :status)
  )
(warn "(jaxon_red-sqp-optimization-instant)~%")

;;contact coordsの位置を関節から少しずらす
(defun setup-jaxon_red-ik-opt-with-larmcontact-face
    (&key (only-kinematics? nil))
  ;; (setq *robot* (instance hrp2jsknts-robot :init))
  (setq *robot* (instance jaxon_red-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "RLEG_LINK5")
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "LLEG_LINK5")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_LINK7")
                        :coords (send (send (send *robot* :link "RARM_LINK7") :copy-worldcoords)
                                      :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_LINK2")
                        :coords ;; (send
                        ;; (send (send (send (send *robot* :link "LARM_LINK3") :copy-worldcoords) :rotate (deg2rad 90) :y) :rotate (deg2rad 30) :x)
                        ;; (send (send (send (send (send *robot* :link "LARM_LINK4") :copy-worldcoords)
                        ;;                   :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z) :rotate (deg2rad -50) :x)
                        (send (send (send (send (send (send *robot* :link "LARM_LINK3") :copy-worldcoords) :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z) :rotate (deg2rad 0) :x) :translate  #f(0 0 0))
                        ;; (send (send (send (send *robot* :link "LARM_LINK5") :copy-worldcoords) :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z)
                                      ;; :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345))
                                      ;; )
                        ))
    (send *robot* :put :larm-elbow-contact-coords
        (make-cascoords :name :larm-elbow-contact-coords
                        :parent (send *robot* :link "LARM_LINK3")
                        :coords ;; (send
                        ;; (send (send (send (send *robot* :link "LARM_LINK3") :copy-worldcoords) :rotate (deg2rad 90) :y) :rotate (deg2rad 30) :x)
                        ;; (send (send (send (send (send *robot* :link "LARM_LINK4") :copy-worldcoords)
                        ;;                   :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z) :rotate (deg2rad -50) :x)
                        (send (send (send (send (send (send *robot* :link "LARM_LINK4") :copy-worldcoords) :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z) :rotate (deg2rad 0) :x) :translate  #f(0 0 0))
                        ;; (send (send (send (send *robot* :link "LARM_LINK5") :copy-worldcoords) :rotate (deg2rad 90) :x) :rotate (deg2rad -90) :z)
                                      ;; :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345))
                                      ;; )
                        ))
  
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless only-kinematics?
    ;; create support polygon
    ;; (create-support-polygon *robot* (send *robot* :hand :rarm :link "RARM_LINK7"))
    ;; (create-support-polygon *robot* (send *robot* :hand :larm :link "LARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "RARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "LARM_LINK3"))
    (create-support-polygon *robot* (send *robot* :link "LARM_LINK4"))
    (create-support-polygon *robot* (send *robot* :link "RLEG_LINK5"))
    (create-support-polygon *robot* (send *robot* :link "LLEG_LINK5"))
    )
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;;set narrow limit of min/max angle instead of min-max table
  ;; (send (send *robot* :joint "RLEG_JOINT0") :min-angle -35)
  ;; (send (send *robot* :joint "LLEG_JOINT0") :max-angle 35)
  ;; (send (send *robot* :joint "RLEG_JOINT2") :min-angle -70)
  ;; (send (send *robot* :joint "LLEG_JOINT2") :min-angle -70)
  (send (send *robot* :joint "HEAD_JOINT1") :max-angle 18)
  
  (send (send *robot* :joint "CHEST_JOINT1") :min-angle 0)
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 30)

  (send (send *robot* :joint "CHEST_JOINT2") :max-angle 30)
  (send (send *robot* :joint "CHEST_JOINT2") :min-angle -30)
  
  (send (send *robot* :joint "RLEG_JOINT0") :max-angle 20)
  (send (send *robot* :joint "LLEG_JOINT0") :min-angle -20)
  (send (send *robot* :joint "RLEG_JOINT3") :max-angle 60)
  (send (send *robot* :joint "LLEG_JOINT3") :max-angle 60)
  
  (send (send *robot* :joint "RARM_JOINT0") :min-angle 0)
  (send (send *robot* :joint "RARM_JOINT0") :max-angle 60)
  (send (send *robot* :joint "LARM_JOINT0") :min-angle -40)
  (send (send *robot* :joint "LARM_JOINT0") :max-angle 0)
  ;;肩の特異点問題dem-step-simulator
  ;; (send (send *robot* :joint "RARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "RARM_JOINT1") :max-angle 80)
  ;; (send (send *robot* :joint "LARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "LARM_JOINT1") :max-angle 80)

  (send (send *robot* :joint "RARM_JOINT1") :min-angle -140)
  (send (send *robot* :joint "RARM_JOINT1") :max-angle 140)
  (send (send *robot* :joint "LARM_JOINT1") :min-angle -140)
  (send (send *robot* :joint "LARM_JOINT1") :max-angle 140)

  ;; (send (send *robot* :joint "LARM_JOINT1") :max-angle 30)

  (send (send *robot* :joint "LARM_JOINT4") :max-angle 0)
  ;; (send (send *robot* :joint "RARM_JOINT4") :max-angle 0)
   
  ;; (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT5") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT5") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)

  ;; (send (send *robot* :joint "RARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "RARM_JOINT7") :min-angle -50)
  ;; (send (send *robot* :joint "RARM_JOINT7") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT6") :max-angle 50)
  ;; (send (send *robot* :joint "LARM_JOINT7") :min-angle -50)
  ;; (send (send *robot* :joint "LARM_JOINT7") :max-angle 50) 
  
  )
;;r -50
;; (dive #f(-300 400 -200) #f(-300 -200 100) :deg-l-x 90 :deg-l-y -40 :deg-r-y 40 :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(-300 0 0) :lleg-pos-shift #f(0 0 0) :send? nil)
(defun ik-opt-with-larmcontact
    (rarm-coords
     larm-coords
     &key
     (optimize-torque? t)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (set-kin-scale? nil)
     (only-kinematics? nil)
     (pre-process-func)
     (reset? t)
     (rleg-pos-shift #f(0 0 0))
     (lleg-pos-shift #f(0 0 0))
     (face (make-cube 800 600 20))
     &allow-other-keys
     )
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))  
  (setq *ground-rleg* (make-cube 800 400 20))
  (setq *ground-lleg* (make-cube 800 400 20))
  ;; (setq *ground-larm* (make-cube 800 600 20))
  (setq *ground-larm* face)
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list (deg2rad -70) 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list (deg2rad -70) 0 0))
        )
  (send *ground-larm* :newcoords
        larm-coords)
  
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (send *ground-larm* :set-color (float-vector 0 1 0))
  (setq *rarm-coords* (instance arrow-object :init))
  (send *rarm-coords* :newcoords
        rarm-coords
        )

  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    (setq tilt-y -40)
    ;; (setq *obst* (make-cube 100 10 1000))
    (setq *obst* (make-cube 20 600 1000))
    (send *obst* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 0 0)))
    (send *obst* :rotate (deg2rad tilt-y) :y)
    ;; (send *obst* :rotate (deg2rad 20) :z)
    (send *obst* :set-color (float-vector 1 0 0) 0.5)

    (setq *obst-side* (make-cube 300 20 1000))
    (send *obst-side* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 300 0)))
    (send *obst-side* :rotate (deg2rad tilt-y) :y)
    (send *obst-side* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  ;; (setup-jaxon_red-ik-opt-with-larmcontact :only-kinematics? only-kinematics? ) 
  (setup-jaxon_red-ik-opt-with-larmcontact-face :only-kinematics? only-kinematics? ) ;;contact coords larm を少しずらした

  (send *ik-param* :set-external-coords-list
        (list
         (send *robot-env* :robot :get :rarm-contact-coords)
         (send *robot-env* :robot :get :larm-contact-coords)
         (send *robot-env* :robot :get :rleg-contact-coords)
         (send *robot-env* :robot :get :lleg-contact-coords))
        
        (list
         (send *robot* :rarm :end-coords :copy-worldcoords)
         (send *robot* :larm :end-coords :copy-worldcoords)
         (send *robot* :rleg :end-coords :copy-worldcoords)
         (send *robot* :lleg :end-coords :copy-worldcoords)))
  (setq external-coords-list (send *ik-param* :get-external-coords-list))
  ;;fix-contact nil 足を自由な位置へ
  ;;t contact-coordsの位置へ
  ;;2 その場で
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "rleg")
                     (instance 2d-planar-contact :init :name "lleg")
                     (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose)
      ;; (send *robot-env* :robot :reset-pose)
    )
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *rarm-coords* *ground-larm*) *env-list*  (if avoid-collision? (list *obst* *obst-side*))
                   ))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if (and (not only-kinematics?) optimize-torque?)
                      (send *robot-env* :drive-joint-list) nil)
                  :only-kinematics? only-kinematics?
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :robot :joint-list) :joint-angle)
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト t以外なら左手は平面
                  (cond ((eq fix-contact? t)
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *ground-larm* :copy-worldcoords)
                               (send *ground-larm* :copy-worldcoords);;リンクの長さ分ずらしたほうが良さそう？
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *rarm-coords* :copy-worldcoords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *rarm-coords* :copy-worldcoords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        ;; (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot  :rarm :end-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        (send *robot-env* :robot :get :larm-elbow-contact-coords)
                        ;; (send *robot-env* :robot  :larm :end-coords)          
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords);;リンクの長さ分ずらしたほうが良さそう？
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              (send *robot-env* :contact "larm" :contact-coords);;?????
                              (send *robot-env* :contact "larm" :contact-coords)
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get :larm-contact-coords)
                            (send *robot-env* :robot :get :larm-elbow-contact-coords)
                            ))
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 200
                                 :contact-face
                                        ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 (send *robot-env* :robot :get :larm_link3-support-polygon)
                                 )
                       (instance default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 200
                                 :contact-face
                                        ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 (send *robot-env* :robot :get :larm_link4-support-polygon)
                                 )
                       ))
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            ))
                  :collision-pair-list
                  (if avoid-collision?
                      (append
                       (mapcar #'(lambda (l) (list l *obst* )) (flatten (send *robot-env* :robot :arms :links)))
                       (mapcar #'(lambda (l) (list l *obst-side*)) (flatten (send *robot-env* :robot :arms :links))))
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin  0.1
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;;多分絶対座標で与える
                        ;; (list (float-vector 0 -80 0 0 0 0))
                        ;; (list (float-vector 0 80 0 0 0 0))
                        ;; 力の正負が微妙かも
                        ;;pivot
                        ;; (list (float-vector 0 -50 0 0 0 0))
                        ;; ;;support
                        ;; ;; (list (float-vector 0 0 -50 0 0 0))
                        ;; (list (float-vector -40 0 0 0 0 0))

                        ;; (list 
                        ;;       (float-vector -40 0 -10 0 0 0))
                        external-wrench-list
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        ;;pivot
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords))
                        ;;support
                        ;; (list (send *robot-env* :robot :link "LARM_LINK5"))
                        ;; (list 
                        ;;  (send *robot-env* :robot  :link "LARM_LINK4"))
                        external-coords-list
                        )
                    )
		                    ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 100)
  )


(defun setup-jaxon_red-ik-change-arms
    (&key (only-kinematics? nil))
  (setq *robot* (instance jaxon_red-robot :init))
  ;; assoc contact coords
  (send *robot* :put :rleg-contact-coords
        (make-cascoords :name :rleg-contact-coords
                        :parent (send *robot* :link "RLEG_LINK5")
                        :coords (send (send *robot* :rleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :lleg-contact-coords
        (make-cascoords :name :lleg-contact-coords
                        :parent (send *robot* :link "LLEG_LINK5")
                        :coords (send (send *robot* :lleg-end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :rarm-contact-coords
        (make-cascoords :name :rarm-contact-coords
                        :parent (send *robot* :link "RARM_LINK7")
                        :coords ;; (send (send (send *robot* :link "RARM_LINK7") :copy-worldcoords)
                        ;; :translate (scale 1e3 (float-vector -0.0042 -0.0233 -0.2345)))
                        ;; (send (send *robot* :link "RARM_LINK7") :copy-worldcoords)
                        (send (send *robot* :rarm :end-coords) :copy-worldcoords)
                        ))
  (send *robot* :put :larm-contact-coords
        (make-cascoords :name :larm-contact-coords
                        :parent (send *robot* :link "LARM_LINK7")
                        :coords ;; (send (send (send *robot* :link "LARM_LINK7") :copy-worldcoords)
                        ;; :translate (scale 1e3 (float-vector -0.0042 0.0233 -0.2345)))
                        ;; (send (send *robot* :link "LARM_LINK7") :copy-worldcoords)
                        (send (send *robot* :larm :end-coords) :copy-worldcoords)
                        ))
  (send *robot* :init-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (unless only-kinematics?
    (create-support-polygon *robot* (send *robot* :link "RARM_LINK7"))
    (create-support-polygon *robot* (send *robot* :link "LARM_LINK7") :normal #f(0 0 -1))
    (create-support-polygon *robot* (send *robot* :link "RLEG_LINK5"))
    (create-support-polygon *robot* (send *robot* :link "LLEG_LINK5"))
    )
  ;; disable min-max table
  (dolist (jnt (send *robot* :joint-list))
    (send jnt :set-val 'joint-min-max-table nil)
    (send jnt :set-val 'joint-min-max-target nil)
    )
  ;;set narrow limit of min/max angle instead of min-max table
  ;; (send (send *robot* :joint "RLEG_JOINT0") :min-angle -35)
  ;; (send (send *robot* :joint "LLEG_JOINT0") :max-angle 35)
  ;; (send (send *robot* :joint "RLEG_JOINT2") :min-angle -70)
  ;; (send (send *robot* :joint "LLEG_JOINT2") :min-angle -70)

  ;; (send (send *robot* :joint "HEAD_JOINT1") :max-angle 18)
  (send (send *robot* :joint "CHEST_JOINT0") :min-angle -0.5)
  (send (send *robot* :joint "CHEST_JOINT0") :max-angle 0.5)
  (send (send *robot* :joint "CHEST_JOINT1") :min-angle 0)
  (send (send *robot* :joint "CHEST_JOINT1") :max-angle 20)
  
  (send (send *robot* :joint "RLEG_JOINT0") :min-angle -18)
  (send (send *robot* :joint "LLEG_JOINT0") :max-angle 18)
  ;; (send (send *robot* :joint "RLEG_JOINT3") :max-angle 60)
  ;; (send (send *robot* :joint "LLEG_JOINT3") :max-angle 60)
  (send (send *robot* :joint "RLEG_JOINT3") :min-angle 50)
  (send (send *robot* :joint "LLEG_JOINT3") :min-angle 50)
  
  (send (send *robot* :joint "RARM_JOINT0") :min-angle 0)
  (send (send *robot* :joint "RARM_JOINT0") :max-angle 40)
  (send (send *robot* :joint "LARM_JOINT0") :min-angle -40)
  (send (send *robot* :joint "LARM_JOINT0") :max-angle 0)
  ;;肩の特異点問題dem-step-simulator
  ;; (send (send *robot* :joint "RARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "RARM_JOINT1") :max-angle 80)
  ;; (send (send *robot* :joint "LARM_JOINT1") :min-angle -100)
  ;; (send (send *robot* :joint "LARM_JOINT1") :max-angle 80)

  (send (send *robot* :joint "RARM_JOINT1") :min-angle -140)
  (send (send *robot* :joint "RARM_JOINT1") :max-angle 140)
  (send (send *robot* :joint "LARM_JOINT1") :min-angle -140)
  (send (send *robot* :joint "LARM_JOINT1") :max-angle 140)

;;腕が180度回転するのを防ぐ
  (send (send *robot* :joint "LARM_JOINT3") :min-angle -90)
  (send (send *robot* :joint "LARM_JOINT3") :max-angle 90)
  (send (send *robot* :joint "RARM_JOINT3") :min-angle -90)
  (send (send *robot* :joint "RARM_JOINT3") :max-angle 90)  
  
  )


(defun ik-opt-change-arm
    (arm-coords 
     &key (arm "larm")
     (optimize-torque? (send *ik-param* :get-optimize-torque?))
     (avoid-collision? (send *ik-param* :get-avoid-collision?))
     (apply-external-force? (send *ik-param* :get-apply-external-force?))
     (fix-contact? (send *ik-param* :get-fix-contact?))
     (set-unique-kin-scale? (send *ik-param* :get-set-unique-kin-scale?))     
     (kin-scale-list ;; (list 1.0
                     ;;        1.0
                     ;;        (list 1 1 1 1 1 0.2)
      ;;        0.0)
      (send *ik-param* :get-kin-scale-list))
     (only-kinematics? (send *ik-param* :get-only-kinematics?))
     (pre-process-func)
     (kin-task-value-thre (send *ik-param* :get-kin-task-value-thre))
     (eom-rot-task-value-thre (send *ik-param* :get-eom-rot-task-value-thre))
     (eom-trans-task-value-thre (send *ik-param* :get-eom-trans-task-value-thre))
     (convergence-check-func #'(lambda(sqp)
                                 (if (and (< (norm (send (send sqp :config-task) :kinematics-task-value :update? nil)) kin-task-value-thre)
                                          (< (norm (send (send sqp :config-task) :eom-rot-task-value :update? nil)) eom-rot-task-value-thre)
                                          (< (norm (send (send sqp :config-task) :eom-trans-task-value :update? nil)) eom-trans-task-value-thre)
                                          )
                                     t ;converge
                                   nil)))
     (reset? (send *ik-param* :get-reset?))
     (external-wrench-list   (list  (send *ik-param* :get-rleg-wrench)
                                   (send *ik-param* :get-lleg-wrench)
                                   (send *ik-param* :get-rarm-wrench)
                                   (send *ik-param* :get-larm-wrench)))
     ;; (external-coords-list (send *ik-param* :get-external-coords-list))
     (rleg-pos-shift (send *ik-param* :get-rleg-pos-shift))
     (lleg-pos-shift (send *ik-param* :get-lleg-pos-shift))
     (set-kin-scale? t)
     (external-coords-list (list (send *robot*  :link "LARM_LINK7")))
     (face-arm (make-cube 800 600 20))
     &allow-other-keys
     )
  (let ((inner-width (* 0.45 (abs (- (elt (send *robot* :lleg :end-coords :worldpos) 1) (elt (send *robot* :rleg :end-coords :worldpos) 1))))))
  (format t "external wrench-list=~A~%" external-wrench-list)
  (send *coords* :set-pre-angle-vector (send *robot* :angle-vector))
  (send *coords* :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
  (send *coords* :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
  (send *coords* :set-rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
  (send *coords* :set-larm-coords (send *robot* :larm :end-coords :copy-worldcoords))

  (setq rleg-pos-shift (v+ rleg-pos-shift (float-vector 0 (* -1 (- (* 0.5 (+ inner-width 150)) inner-width)) -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift (float-vector 0 (- (* 0.5 (+ inner-width 150)) inner-width) -10)))
  (format t "rleg-pos-shift in ik = ~A~%" rleg-pos-shift)
  (setq *ground-rleg* (make-cube 500 (+ 150 inner-width) 20)) ;;200
  (setq *ground-lleg* (make-cube 500 (+ 150 inner-width) 20)) ;;200



  
  (if (equal arm "larm")
      (setq oparm "rarm")
    (setq oparm "larm")
    )
  (format t "arm = ~A~%" arm)
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))  

  ;; (setq *ground-rleg* (make-cube 800 400 20))

  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list (deg2rad 0) 0 0)))
  
  ;; (setq *ground-lleg* (make-cube 800 400 20))

  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list (deg2rad 0) 0 0)))

  (setq *ground-arm* (copy-object face-arm))
  (send *ground-arm* :set-color (float-vector 0 1 0))

  (setq com-box (make-cube 100 100 100))
  (send com-box :set-color #f(1 0 0))
  (setq com (send *ground-arm* :centroid))
  (send com-box :translate com)
  (send *ground-arm* :newcoords
        (make-coords :pos com :rpy (list (deg2rad 0) 0 0))
        )
  (format t "ground arm = ~A~%" (send (send *ground-arm* :copy-worldcoords) :worldcoords))  
  (send *ground-arm* :translate (scale -1 com))
  (format t "ground arm = ~A~%" (send (send *ground-arm* :copy-worldcoords) :worldcoords))
  ;; (send *ground-arm* face-arm)
  
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-arm*))
  (send-all *env-list* :worldcoords)

  (setq *arm-coords* (instance arrow-object :init))
  (send *arm-coords* :newcoords (send arm-coords :copy-worldcoords))
  (format t "arm-coords = ~A~%" (send (send arm-coords :copy-worldcoords) :worldpos))

  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
  (when avoid-collision?
    (format t "avoid collision~%")
    (setq *obst* (copy-object face-arm))
    (send *obst* :translate #f(200 0 0))
    (send *obst* :set-color (float-vector 1 1 0))
    )
  
  ;; setup robot
  (setup-jaxon_red-ik-change-arms :only-kinematics? only-kinematics? )
  
  ;; (setq *wall* (elt (send *ground-arm* :faces) 0))
  (setq *wall* (car (send *ground-arm* :get-face :prism :top)))
  (send *wall* :put :normal-move-target-axis :z)
  (send *wall* :put :normal-tolerance 30.0) ;; [deg]
  ;;fix-contact nil 足を自由な位置へ
  ;;t contact-coordsの位置へ
  ;;2 その場で
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "rleg")
                     (instance 2d-planar-contact :init :name "lleg")
                     (generate-contact-ik-arg-from-rect-face :rect-face *wall* :name (format nil "~A" arm)) ;;toutch-wall-ik.lを参考にした
                     ;; (instance 2d-planar-contact :init :name  (format nil "~A" arm)) 
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         ;;接触面定義
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         ;; (send *robot-env* :contact (format nil "~A" arm) :set-from-face :face *wall*)
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose)
    )

  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  
  (objects (append (list *robot* *arm-coords* com-box (send *ik-param* :get-object)) *env-list*  (if avoid-collision? (list *obst*))))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if (and (not only-kinematics?) optimize-torque?)
                      (send *robot-env* :drive-joint-list) nil)
                  :only-kinematics? only-kinematics?
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :robot :joint-list) :joint-angle)
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト t以外なら左手は平面
                  (cond ((eq fix-contact? t)
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *arm-coords* :copy-worldcoords)
                               (send *ground-arm* :copy-worldcoords)
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *arm-coords* :copy-worldcoords)
                               (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *arm-coords* :copy-worldcoords)
                               (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト link
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        (send *robot-env* :robot  (read-from-string (format nil ":~A" oparm)) :end-coords)
                        ;; (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" oparm)))
                        ;; (if (equal arm "larm")
                            ;; (send
                             (send 
                              (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" arm))) ;; :copy-worldcoords)
                             :rotate (deg2rad 90) :x)
                          ;; (send (send (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" arm))) :copy-worldcoords) :rotate (deg2rad -90) :x))
                             )
                  ;; :kin-scale-list
                  ;; (if set-kin-scale? ;;1だとxyz rpy全方向に動ける、０だと元のまま動けない
                  ;;     (list 1.0
                  ;;           1.0
                  ;;           1.0
                  ;;           (list 1 1 1 0 0 1)
                  ;;           ;; 1.0
                  ;;           ))

                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-arm* :copy-worldcoords)
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              (send *robot-env* :contact (format nil "~A" arm) :contact-coords)
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            ;; (if (equal arm "larm")
                                ;; (send
                                 ;; (send
                                  (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" arm))) ;; :copy-worldcoords)
                                      ;; :rotate (deg2rad 90) :x)
                              ;; (send (send (send *robot-env* :robot :get (read-from-string (format nil ":~A-contact-coords" arm))) :copy-worldcoords) :rotate (deg2rad -90) :x)
                              ;; )
                            )
                    )
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       ;; (instance* default-contact-constraint
                       ;;            :init
                       ;;            :mu-trans 0.2
                       ;;            :mu-rot 0.01
                       ;;            :max-fz 40
                       ;;            (let* ((poly
                       ;;                    (send *robot-env* :robot :get :larm_link7-support-polygon))
                       ;;                   (vs
                       ;;                    (mapcar #'(lambda (v)
                       ;;                                (send (send (send *robot-env* :robot :get :larm-contact-coords) :worldcoords)
                       ;;                                      :inverse-transform-vector v))
                       ;;                            (send poly :vertices)
                       ;;                            ))
                       ;;                   )
                       ;;              (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                       ;;                    :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                       ;;                    :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                       ;;                    :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                       ;;                    ))
                       ;;            )
                       (instance default-contact-constraint
                                 :init-from-support-polygon :support-polygon nil)
                       
                       ;; )
                    ))
                  :kin-scale-list
                                          (list
                         ;; 1.0
                         ;; 1.0
                         (list 1 1 1 1 1 1)
                         (list 1 1 1 1 1 1)
			 1.0
                         (list 1 1 1 0 0 1)
                         (list 1 1 1 1 0.5 1))
                  ;; (if set-unique-kin-scale?
                  ;;     kin-scale-list ;;足平は前を向くように
                  ;;     (cond
                  ;;      ((equal arm "both")
                  ;;       (list
                  ;;        ;; 1.0
                  ;;        ;; 1.0
                  ;;        (list 1 1 1 1 1 1)
                  ;;        (list 1 1 1 1 1 1)
                  ;;        ;; (list 1 1 1 1 1 0.5)
                  ;;        ;; (list 1 1 1 1 1 0.5)
                  ;;        1.0
                  ;;        1.0))
                  ;;      ((equal arm "rarm")
                  ;;       (list
                  ;;        ;; 1.0
                  ;;        ;; 1.0
                  ;;        (list 1 1 1 1 1 1)
                  ;;        (list 1 1 1 1 1 1)
                  ;;        ;; (list 1 1 1 1 0.5 1)
                  ;;        (list 1 1 1 0 0 1)
                  ;;        0.0))
                  ;;      ((equal arm "larm")
                  ;;       (list
                  ;;        ;; 1.0
                  ;;        ;; 1.0
                  ;;        (list 1 1 1 1 1 1)
                  ;;        (list 1 1 1 1 1 1)
                  ;;        (list 1 1 1 0 0 1)
                  ;;        (list 1 1 1 1 0.5 1)))))
                  :collision-pair-list
                  (if avoid-collision?
                      ;; (list 
                       ;; (flatten (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
                       (append (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links)))
                       (list (list (send *robot* :link "LLEG_LINK2") (send *robot* :link "LARM_LINK7")))
                       ;; (list (send *robot* :link "LLEG_LINK1") (send *robot* :link "LARM_LINK6"))
                       (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "RARM_LINK7")))
                       ;; (list (send *robot* :link "RLEG_LINK1") (send *robot* :link "RARM_LINK6"))
                       ;; (list (send *robot* :link "LLEG_LINK1") (send *robot* :link "LARM_LINK7"))
                       ;; (list (send *robot* :link "RLEG_LINK1") (send *robot* :link "RARM_LINK7"))
                       )
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin  0.01
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        external-wrench-list
                        ;; (list 
                        ;;       ;; (float-vector 0 0 -80 0 0 0)
                        ;;       (float-vector -40 0 -10 0 0))
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        external-coords-list
                        ;; (list 
                        ;;       ;; (send *robot-env* :robot :get :larm-contact-coords)
                        ;;       (send *robot-env* :robot  :link "LARM_LINK7"))
                        )
                    )
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  :no-print? nil
                  :convergence-check-func
                  convergence-check-func
                  :failure-callback-func
                  #'(lambda(val)
                      (print "fail"))
                  ))
  (send *sqp-opt* :optimize :loop-num 100)
  (send (send *robot* :joint "RLEG_JOINT3") :min-angle 0)
  (send (send *robot* :joint "LLEG_JOINT3") :min-angle 0)
  (if (equal (send *sqp-opt* :status) :not-converged)
      (progn
        (format t "ik failed~%")
        (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
        (send *robot* :move-coords (send *coords* :get-lleg-coords) (send *robot* :lleg :end-coords))
        nil
        )
    t)
  )
  )

;;progn (load "demo.l") (prepare-simulator)  (send *robot* :angle-vector #f(-5.80758 -6.03765 -75.0039 57.4591 -13.1998 9.56411 8.66658 8.42425 -76.3062 57.8698 -12.498 -10.2217 2.00262 3.11233 -2.03515 2.71054 -7.51743 6.97288 28.0959 -132.366 -31.2662 -94.6359 -34.0954 -10.6936 30.1544 -8.20333 23.1193 125.023 26.9899 -91.5323 29.6232 5.47784 26.1095)) (start-coords) 
;; (dive-larm-only #f(-300 -200 100) :deg-l-x 90 :deg-l-y -40  :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(-300 0 0) :lleg-pos-shift #f(0 0 0) :send? nil)

;;  (dive-larm-only #f(-300 -200 100) :deg-l-x 90 :deg-l-y -40  :reset? t :fix-contact? nil :wait? t :avoid-collision? t  :rleg-pos-shift #f(-300 0 0) :lleg-pos-shift #f(0 0 0) :send? nil)
(defun ik-opt-with-larmcontact-only
    (
     larm-coords
     &key
     (optimize-torque? t)
     (avoid-collision? t)
     (apply-external-force? nil)
     (fix-contact? nil)
     (set-kin-scale? nil)
     (only-kinematics? nil)
     (pre-process-func)
     (reset? t)
     (rleg-pos-shift #f(0 0 0))
     (lleg-pos-shift #f(0 0 0))
     &allow-other-keys
     )
  (setq rleg-pos-shift (v+ rleg-pos-shift #f(0 0 -10)))
  (setq lleg-pos-shift (v+ lleg-pos-shift #f(0 0 -10)))  
  (setq *ground-rleg* (make-cube 800 400 20))
  (setq *ground-lleg* (make-cube 800 400 20))
  (setq *ground-larm* (make-cube 800 600 20))
  (send *ground-rleg* :newcoords
        (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate rleg-pos-shift) :worldpos)  :rpy (list (deg2rad -70) 0 0))
        )
  (send *ground-lleg* :newcoords
        (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate lleg-pos-shift) :worldpos)  :rpy (list (deg2rad -70) 0 0))
        )
  (send *ground-larm* :newcoords
        larm-coords)
  
  (setq *env-list* (list *ground-rleg* *ground-lleg* *ground-larm*))
  (send-all *env-list* :worldcoords)
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))
  (send *ground-larm* :set-color (float-vector 0 1 0))

  (setq rleg-now (send *robot* :rleg :end-coords :copy-worldcoords))
  (setq lleg-now (send *robot* :lleg :end-coords :copy-worldcoords))
 
 (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    (setq tilt-y -40)
    ;; (setq *obst* (make-cube 100 10 1000))
    (setq *obst* (make-cube 20 600 1000))
    (send *obst* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 0 0)))
    (send *obst* :rotate (deg2rad tilt-y) :y)
    ;; (send *obst* :rotate (deg2rad 20) :z)
    (send *obst* :set-color (float-vector 1 0 0) 0.5)

    (setq *obst-side* (make-cube 300 20 1000))
    (send *obst-side* :translate  (v+ (send (send larm-coords :copy-worldcoords) :worldpos) #f(150 300 0)))
    (send *obst-side* :rotate (deg2rad tilt-y) :y)
    (send *obst-side* :set-color (float-vector 1 0 0) 0.5)
    )
  ;; setup robot
  (setup-jaxon_red-ik-opt-with-larmcontact-face :only-kinematics? only-kinematics? )
  ;;fix-contact nil 足を自由な位置へ
  ;;t contact-coordsの位置へ
  ;;2 その場で
  (cond (fix-contact?
         (setq *robot-env*
               (instance robot-environment :init :robot *robot*))
         )
        (t
         (setq *contact-list*
               (list (instance 2d-planar-contact :init :name "rleg")
                     (instance 2d-planar-contact :init :name "lleg")
                     (instance 2d-planar-contact :init :name "larm")
                     ))
         (setq *robot-env*
               (instance robot-environment :init :robot *robot* :contact-list *contact-list*))
         (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
         (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
         (send *robot-env* :contact "larm" :set-from-face :face (elt (send *ground-larm* :faces) 1))
         (send *robot-env* :contact "rleg" :rotate pi :z :local)
         (send *robot-env* :contact "lleg" :rotate pi :z :local)
         ))

  (if reset?
      (send *robot-env* :robot :reset-manip-pose)
      ;; (send *robot* :torso-waist-p :joint-angle 30)
      )
  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (objects (append (list *robot* *ground-larm*) *env-list*  (if avoid-collision? (list *obst* *obst-side*))
                   ))

  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env *robot-env*
                  :drive-joint-list
                  (if (and (not only-kinematics?) optimize-torque?)
                      (send *robot-env* :drive-joint-list) nil)
                  :only-kinematics? only-kinematics?
                  :posture-joint-list
                  (send *robot-env* :robot :joint-list)
                  :posture-joint-angle-list
                  (send-all (send *robot-env* :robot :joint-list) :joint-angle)
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト t以外なら左手は平面
                  (cond ((eq fix-contact? t)
                         (list (send *ground-rleg* :copy-worldcoords)
                               (send *ground-lleg* :copy-worldcoords)
                               (send *ground-larm* :copy-worldcoords)
                               (send *ground-larm* :copy-worldcoords);;リンクの長さ分ずらしたほうが良さそう？
                               )
                         )
                        ((eq fix-contact? nil)
                         (list (send *robot-env* :contact "rleg" :contact-coords)
                               (send *robot-env* :contact "lleg" :contact-coords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         )
                        (t
                         (list rleg-now
                               lleg-now
                               (send *robot-env* :contact "larm" :contact-coords);;?????
                               (send *robot-env* :contact "larm" :contact-coords)
                               )
                         ))
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (list (send *robot-env* :robot :get :rleg-contact-coords)
                        (send *robot-env* :robot :get :lleg-contact-coords)
                        ;; (send *robot-env* :robot :get :rarm-contact-coords)
                        (send *robot-env* :robot :get :larm-contact-coords)
                        (send *robot-env* :robot :get :larm-elbow-contact-coords)
                        ;; (send *robot-env* :robot  :larm :end-coords)          
                        )
                  :contact-target-coords-list
                  (if (not only-kinematics?)
                      (cond ((eq fix-contact? t)
                             (list (send *ground-rleg* :copy-worldcoords)
                                   (send *ground-lleg* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords)
                                   (send *ground-larm* :copy-worldcoords);;リンクの長さ分ずらしたほうが良さそう？
                                   )
                             )
                            ((eq fix-contact? nil)
                             (list (send *robot-env* :contact "rleg" :contact-coords)
                                   (send *robot-env* :contact "lleg" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   (send *robot-env* :contact "larm" :contact-coords)
                                   )
                             )
                            (t
                             (list
                              rleg-now
                              lleg-now
                              (send *robot-env* :contact "larm" :contact-coords);;?????
                              (send *robot-env* :contact "larm" :contact-coords)
                              )
                         )))
                  :contact-attention-coords-list
                  (if (not only-kinematics?)
                      (list (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get :larm-contact-coords)
                            (send *robot-env* :robot :get :larm-elbow-contact-coords)
                            ))
                  :contact-constraint-list
                  (if (not only-kinematics?)
                      (list
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance* default-contact-constraint
                                  :init
                                  :mu-trans 0.2
                                  :mu-rot 0.01
                                  :max-fz 600
                                  (let* ((poly
                                          (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                         (vs
                                          (mapcar #'(lambda (v)
                                                      (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                            :inverse-transform-vector v))
                                                  (send poly :vertices)
                                                  ))
                                         )
                                    (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                          :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                          :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                          :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)
                                          ))
                                  )
                       (instance default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 200
                                 :contact-face
                                        ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 (send *robot-env* :robot :get :larm_link4-support-polygon)
                                 )
                       (instance default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 200
                                 :contact-face
                                        ;                                 (send *robot-env* :robot :get :larm_link6-support-polygon)
                                 (send *robot-env* :robot :get :larm_link4-support-polygon)
                                 )
                       ))
                  :kin-scale-list
                  (if set-kin-scale?
                      (list 1.0
                            10.0
                            (list 1 2 3 4 5 6)
                            (list 1 2 3 4 5 6)
                            ))
                  :collision-pair-list

                  (if avoid-collision?
                      (append
                      (append
                       (mapcar #'(lambda (l) (list l *obst* )) (flatten (send *robot-env* :robot :arms :links)))
                       (mapcar #'(lambda (l) (list l *obst-side*)) (flatten (send *robot-env* :robot :arms :links))))
                       (list (list (send *robot-env* :robot :link "RARM_LINK7") (send *robot-env* :robot :link "RLEG_LINK2"))))
                       ;; (mapcar #'(lambda (l) (list l (send *robot-env* :robot :link "RARM_LINK6") ) (flatten (send *robot-env* :robot :legs :links)))))
                  )
                  
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin 0.05
                  :external-wrench-list
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list 
                              ;; (float-vector 0 0 -80 0 0 0)
                              (float-vector -80 0 -80 0 0 0))
                        ;;pivot
                        ;; (list (float-vector 0 -50 0 0 0 0))
                        ;; ;;support
                        ;; ;; (list (float-vector 0 0 -50 0 0 0))
                        ;; (list (float-vector -40 0 0 0 0 0))
                        ))
                  :external-coords-list  ;;外レンチ作用点リスト
                  (if (and (not only-kinematics?) apply-external-force?)
                      (progn
                        (list 
                              ;; (send *robot-env* :robot :get :larm-contact-coords)
                              (send *robot-env* :robot  :link "LARM_LINK4"))
                        ;;pivot
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords))
                        ;;support
                        ;; (list (send *robot-env* :robot :link "LARM_LINK5"))
                        )
                    )
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func
                  pre-process-func
                  :post-process-func
                  #'(lambda (sqp)
                      (when (= (mod (send sqp :iteration) 20) 0)
                        (validate-element-jacobian :config-task (send sqp :config-task))
                        ;; (validate-jacobian :config-task (send sqp :config-task))
                        ))
                  ))
  (send *sqp-opt* :optimize :loop-num 100)
  )



(defun ik-walk
    (rarm-coords larm-coords
                 &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1))
                 (optimize-torque? t)
                 (pre-process-func)
                 (post-process-func)
                 (stop 50)
                 )
  (if (eq rleg nil)
      (setq rleg  (send *coords* :get-rleg))
    (format t "rleg ~A" rleg))
  (if (eq lleg nil)
      (setq lleg  (send *coords* :get-lleg)))
  (if (eq cog nil)
      (setq cog  (send *coords* :get-cog)))

  (send *robot* :torso-waist-p :joint-angle 30)
  (setq *target-centroid-pos* (float-vector 300 0 0))  
    ;;ground rleg
  (setq *ground* (make-cube 3000 2000 5))  
  (send *ground* :newcoords
        (make-coords :pos (float-vector 0 -1200 0))); -923.285)));(- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
                                        ;        (send (send *pull_target* :copy-worldcoords) :translate #f(0 -200 0))) ;;thickness of door
        
  (send *ground* :set-color (float-vector 1.0 0 0))

  (setq *floor1* (elt (send *ground* :faces) 3)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground* :faces))
  (send *floor1* :put :margin 20) ;; [mm]
  (send *floor1* :put :normal-move-target-axis :z)  
  (send *floor1* :name :face-rleg)

  ;;ground2 lleg
  (setq *ground2* (make-cube 3000 2000 5))  
  (send *ground2* :newcoords
        (make-coords :pos (float-vector 0 1200 0)));;-923.285))); (- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
  (send *ground2* :set-color (float-vector 0 0 1.0))  
  (setq *floor2* (elt (send *ground2* :faces) 3)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground2* :faces))
  (send *floor2* :put :margin 20) ;; [mm]
  (send *floor2* :put :normal-move-target-axis :z)  
  (send *floor2* :name :face-lleg)

  (send *robot*
        :inverse-kinematics-optmotiongen

        (list
         ;; rleg
         ;; lleg
         (send (send *robot* :rleg :end-coords) :copy-worldcoords)
         (send (send *robot* :lleg :end-coords) :copy-worldcoords)
         ;; *floor1*
         ;; *floor2*
         ;; (send *coords* :get-rleg)
         ;; (send *coords* :get-lleg)         

              rarm-coords
              larm-coords
              ;; (make-coords :pos *target-centroid-pos*)              
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :larm :end-coords)
              ;; :cog

              )
        :link-list
        (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :rarm :end-coords :parent))                 
              (send *robot* :link-list (send *robot* :larm :end-coords :parent)))
        
        :contact-target-coords
        (list ;; (send (send *robot* :rleg :end-coords) :copy-worldcoords)
         ;; (send (send *robot* :lleg :end-coords) :copy-worldcoords)
         *floor1*
         *floor2*
              )
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              )
        ;; (list (send *coords* :get-rleg )
        ;;       (send *coords* :get-lleg )
        ;;       )
                

        :contact-constraint
        (list
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send  (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                   )

         )
        :translation-axis (list :xy :xy t t)
        :rotation-axis (list t t t t)
        ;; :debug-view t
        :root-virtual-mode :6dof
        :collision-avoidance-link-pair
        (list
         (list 
         (elt (send *robot* :lleg :links) 5)
         (elt (send *robot* :rleg :links) 5)
         )
         )
        :collision-distance-margin 100
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        ;;cog関係は無視されてそう
        ;; :target-centroid-pos (v+ (midpoint 0.5 (send rleg :worldpos) (send lleg :worldpos))
        ;;                          cog)
        :target-centroid-pos (v+ (midpoint 0.5 (send (send (send *robot* :rleg :end-coords) :copy-worldcoords) :worldpos)
                                           (send (send (send *robot* :lleg :end-coords) :copy-worldcoords) :worldpos))
                                 #f(1000 0 0))

        :cog-translation-axis move-root-link     
        )
  
  )

(defun ik-st
    (rarm-coords larm-coords
                 &key rleg lleg cog (move-root-link :z) (virtual-joint-weight #f(0.5 0.5 0.5  1 1 1))
                 (optimize-torque? t)
                 (pre-process-func)
                 (post-process-func)
                 (stop 30)
                 )

  (if (eq rleg nil)
      (setq rleg  (send *coords* :get-rleg))
    (format t "rleg ~A" rleg))
  (if (eq lleg nil)
      (setq lleg  (send *coords* :get-lleg)))
  (if (eq cog nil)
      (setq cog  (send *coords* :get-cog)))

  ;; (create-support-polygon *robot* (send (send *coords* :get-rleg) :parent))
  ;; (create-support-polygon *robot* (send (send *robot* :lleg :end-coords) :parent))  

  ;;ground rleg
  (setq *ground* (make-cube 3000 2000 5))  
  (send *ground* :newcoords
        (make-coords :pos (float-vector 0 0 0))); -923.285)));(- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
                                        ;        (send (send *pull_target* :copy-worldcoords) :translate #f(0 -200 0))) ;;thickness of door
        
  (send *ground* :set-color (float-vector 1.0 0 0))

  (setq *floor1* (elt (send *ground* :faces) 1)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground* :faces))
  (send *floor1* :put :margin 20) ;; [mm]
  (send *floor1* :put :normal-move-target-axis :z)  
  (send *floor1* :name :face-rleg)

  ;;ground2 lleg
  (setq *ground2* (make-cube 3000 2000 5))  
  (send *ground2* :newcoords
        (make-coords :pos (float-vector 0 0 0)));;-923.285))); (- (- 0 (elt (send *robot* :body_lk :worldpos ) 2)) 2.5))))
  (send *ground2* :set-color (float-vector 0 0 1.0))  
  (setq *floor2* (elt (send *ground2* :faces) 1)) ;;elt 3 :normal-move-target-axis :z)  が見た感じ最適？ 
  (print (send *ground2* :faces))
  (send *floor2* :put :margin 20) ;; [mm]
  (send *floor2* :put :normal-move-target-axis :z)  
  (send *floor2* :name :face-lleg)


  (send *robot*
        :inverse-kinematics-statics-optmotiongen
        ;;:inverse-kinematics-trajectory-optmotiongen
        (list
         rleg
         lleg
         ;; *floor1*
         ;; *floor2*
              rarm-coords
              larm-coords
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :rarm :end-coords)
              (send *robot* :larm :end-coords)
              )
        :contact-target-coords
        (list (send (send *robot* :rleg :end-coords) :copy-worldcoords)
              (send (send *robot* :lleg :end-coords) :copy-worldcoords)
              )
        ;; (list *floor1* *floor2*)
        :contact-move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              )
        :contact-constraint
        (list
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :rleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :rleg :end-coords) :worldcoords)
                   )
         (instance default-contact-constraint
                   :init-from-support-polygon
                   :support-polygon (send (send (send *robot* :lleg :end-coords) :parent) :get :support-polygon)
                   :support-polygon-coords (send (send *robot* :lleg :end-coords) :worldcoords)
                   )
         )
     
        :translation-axis (list t t t t)
        :rotation-axis (list t t t t)
        ;; :translation-axis (list t t )
        ;; :rotation-axis (list t t )
        
        :debug-view t
;;        :root-virtual-mode :6dof ;;root fix version
        :root-virtual-mode :6dof ;:planar  ;;virtual joint version
        :optimize-torque? optimize-torque?
        :stop stop
        :min-loop 30
;;??        :root-virtual-joint-invariant? t        
        :pre-process-func pre-process-func
        :post-process-func post-process-func
        )
  )
;; #f(-1.76859 -6.82574 -46.7277 85.9558 -39.4514 6.83213 0.648952 9.15973 -44.2373 85.3872 -41.3745 -9.1629 -4.58023 21.1258 1.08981 -1.38348 -5.0 37.0 16.9453 -29.5958 -4.01948 -73.3314 48.134 -13.0157 20.4156 -37.0 21.3996 33.0076 6.06592 -71.559 -44.4047 15.882 17.5909)
