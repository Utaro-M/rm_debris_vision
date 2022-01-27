(load "package://eus_qp/optmotiongen/euslisp/sqp-optimization.l")
(load "package://eus_qp/optmotiongen/euslisp/instant-configuration-task.l")
(load "package://eus_qp/optmotiongen/euslisp/validate-jacobian.l")
;; (require "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require "package://eus_qp/optmotiongen/euslisp/inverse-kinematics-statics-wrapper.l")
(require "package://rm_debris_vision/euslisp/params.l")
(print "load my-ik-latest.l")

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
  ;; (send (send *robot* :joint "RLEG_JOINT3") :min-angle 90)
  ;; (send (send *robot* :joint "LLEG_JOINT3") :min-angle 90)
  

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

(defun display-ik-opt-solver ()
  ;;display
  (setq inner-width (* 0.45 (abs (- (elt (send *robot* :lleg :end-coords :worldpos) 1) (elt (send *robot* :rleg :end-coords :worldpos) 1)))))
  (setq *ground-rleg* (make-cube 500 (+ 250 inner-width) 20)) ;;200
  (setq *ground-lleg* (make-cube 500 (+ 250 inner-width) 20)) ;;200
  (send *ground-rleg* :newcoords (send (send *ik-opt-param* :get-my-coords) :get-ground-rleg))
  (send *ground-lleg* :newcoords (send (send *ik-opt-param* :get-my-coords) :get-ground-lleg))
  (setq *env-list* (list *ground-rleg* *ground-lleg*))
  (send-all *env-list* :worldcoords)
  (send *ground-rleg* :set-color (float-vector 1 0 0))
  (send *ground-lleg* :set-color (float-vector 0 0 1))

  (setq *rarm-coords* (instance arrow-object :init))
  (send *rarm-coords* :newcoords (send (send *ik-opt-param* :get-my-coords) :get-rarm-coords))
  (setq *larm-coords* (instance arrow-object :init))
  (send *larm-coords* :newcoords (send (send *ik-opt-param* :get-my-coords) :get-larm-coords))

  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )
  (setq display-list (list *robot*))
  (when (send *ik-opt-param* :get-avoid-collision?)
    (format t "avoid collision~%")
    ;; (if heave?
    ;;     (send *obst* :translate (v+ (scale 0.5 (v+ (send rarm-coords :worldpos) (send larm-coords :worldpos))) #f(-30 0 -400)))
    ;;   (send *obst* :translate (v+ (scale 0.5 (v+ (send rarm-coords :worldpos) (send larm-coords :worldpos))) #f(0 0 -400))))
    ;; (send *obst* :set-color (float-vector 0 1 1) 0.5)
    (setq display-list (append display-list (send *ik-opt-param* :get-obstacle)))
    )
  (objects display-list)
  )

(defun ik-opt-solver (&key (*ik-opt-param* *ik-opt-param*))
  (display-ik-opt-solver)
  (if (send *ik-opt-param* :get-reset?)
      (send (send *ik-opt-param* :get-robot-env) :robot :reset-manip-pose))

  ;; setup robot
  (setup-jaxon_red-ik-opt :only-kinematics? (send *ik-opt-param* :get-only-kinematics?))
  (setq *instant-config-task*
        (instance instant-configuration-task :init
                  :robot-env (send *ik-opt-param* :get-robot-env)
                  :drive-joint-list (send *ik-opt-param* :get-drive-joint-list)
                  :only-kinematics? (send *ik-opt-param* :get-only-kinematics?)
                  :posture-joint-list
                  (send *ik-opt-param* :get-posture-joint-list)
                  :posture-joint-angle-list
                  (send *ik-opt-param* :get-posture-joint-angle-list)
                  :kin-target-coords-list ;;幾何到達目標位置姿勢リスト
                  (send *ik-opt-param* :get-kin-target-coords-list)
                  :kin-attention-coords-list ;;幾何到達着目姿勢リスト
                  (send *ik-opt-param* :get-kin-attention-coords-list)
                  :contact-target-coords-list
                  (send *ik-opt-param* :get-contact-target-coords-list)
                  :contact-attention-coords-list
                  (send *ik-opt-param* :get-contact-attention-coords-list)
                  :contact-constraint-list
                  (send *ik-opt-param* :get-contact-constraint-list)
                  :kin-scale-list
                  (send *ik-opt-param* :get-kin-scale-list)
                  :collision-pair-list (send *ik-opt-param* :get-collision-pair-list)
                  ;; (list  (send *robot-env* :robot :rleg :links)  (send *robot-env* :robot :lleg :links))
                  :collision-distance-margin (send *ik-opt-param* :get-collision-distance-margin)
                  :external-wrench-list (send *ik-opt-param* :get-external-wrench-list)
                  :external-coords-list  ;;外レンチ作用点リスト
                  (send *ik-opt-param* :get-external-coords-list)
                  ))

  (setq *sqp-opt*
        (instance sqp-optimization :init
                  :config-task *instant-config-task*
                  :pre-process-func (send *ik-opt-param* :get-pre-process-func)
                  :post-process-func #'(lambda (sqp)
                                         (when (= (mod (send sqp :iteration) 20) 0)
                                           (validate-element-jacobian :config-task (send sqp :config-task))))
                                           ;; (validate-jacobian :config-task (send sqp :config-task))
                  :no-print? nil
                  :convergence-check-func
                  #'(lambda(sqp)
                     (if (and (< (norm (send (send sqp :config-task) :kinematics-task-value :update? nil)) (send *ik-opt-param* :get-kin-task-value-thre))
                              (< (norm (send (send sqp :config-task) :eom-rot-task-value :update? nil)) (send *ik-opt-param* :get-eom-rot-task-value-thre))
                              (< (norm (send (send sqp :config-task) :eom-trans-task-value :update? nil)) (send *ik-opt-param* :get-eom-trans-task-value-thre)))
                         t ;converge
                       nil));; (send *ik-opt-param* :get-convergence-check-func)
                  :failure-callback-func #'(lambda(val)
                                             (print "fail"))))
  (send *sqp-opt* :optimize :loop-num 200)
  (if (equal (send *sqp-opt* :status) :not-converged)
      (progn
        (format t "ik failed~%")
        (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
        (send *robot* :move-coords (send *coords* :get-lleg-coords) (send *robot* :lleg :end-coords))
        nil
        )
    (setq *robot* (send (send *ik-opt-param* :get-robot-env) :robot))
  ;; (send (send *robot* :joint "RLEG_JOINT3") :min-angle 0)
  ;; (send (send *robot* :joint "LLEG_JOINT3") :min-angle 0)
  )
