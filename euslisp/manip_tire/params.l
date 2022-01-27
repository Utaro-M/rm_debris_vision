(require "package://rm_debris_vision/euslisp/manip_tire/my-ik-latest.l")
(defclass my-coords
  :super propertied-object
  :slots (rarm-coords larm-coords rleg-coords lleg-coords rleg-new-coords lleg-new-coords rarm-target-coords larm-target-coords pre-angle-vector ground-rleg ground-lleg)
  )

(defmethod my-coords
  (:init () ;;rleg-support-polygon lleg-support-polygon)
         (progn
           (send self :set-rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords))
           (send self :set-larm-coords (send *robot* :larm :end-coords :copy-worldcoords))
           (send self :set-rleg-coords (send *robot* :rleg :end-coords :copy-worldcoords))
           (send self :set-lleg-coords (send *robot* :lleg :end-coords :copy-worldcoords))
           (send self :set-rleg-new-coords (send *robot* :rleg :end-coords :copy-worldcoords))
           (send self :set-lleg-new-coords (send *robot* :lleg :end-coords :copy-worldcoords))
           (send self :set-pre-angle-vector (send *robot* :angle-vector))
           (send self :set-latest-angle-vector (send *robot* :angle-vector))
           (send self :set-ground-rleg (make-coords :pos (send (send (send *robot* :rleg :end-coords :copy-worldcoords) :translate #f(0.0 -80.0 -20.0)) :worldpos)  :rpy #f(0 0 0)))
           (send self :set-ground-lleg (make-coords :pos (send (send (send *robot* :lleg :end-coords :copy-worldcoords) :translate #f(0.0 80.0 -20.0)) :worldpos)  :rpy #f(0 0 0)))
           )
         self)
  (:set-rarm-coords (val)             (setq rarm-coords val))
  (:set-larm-coords (val)		      (setq larm-coords val))
  (:set-rleg-coords (val)		      (setq rleg-coords val))
  (:set-lleg-coords (val)		      (setq lleg-coords val))
  (:set-rleg-new-coords (val)		  (setq rleg-new-coords val))
  (:set-lleg-new-coords (val)		  (setq lleg-new-coords val))
  (:set-rarm-target-coords (val)      (setq rarm-target-coords val))
  (:set-larm-target-coords (val)      (setq larm-target-coords val))
  (:set-pre-angle-vector (val)        (setq pre-angle-vector val))
  (:set-latest-angle-vector (val)     (setq latest-angle-vector val))
  (:set-ground-rleg (val)            (setq ground-rleg val))
  (:set-ground-lleg (val)            (setq ground-lleg val))

  (:get-rarm-coords ()                 rarm-coords)
  (:get-larm-coords ()                 larm-coords)
  (:get-rleg-coords ()                 rleg-coords)
  (:get-lleg-coords ()		           lleg-coords)
  (:get-lleg-new-coords ()			   lleg-new-coords)
  (:get-rleg-new-coords ()			   rleg-new-coords)
  (:get-rarm-target-coords ()          rarm-target-coords)
  (:get-larm-target-coords ()          larm-target-coords)
  (:get-pre-angle-vector ()            pre-angle-vector)
  (:get-latest-angle-vector ()         latest-angle-vector)
  (:get-ground-rleg ()                 ground-rleg )
  (:get-ground-lleg ()                 ground-lleg )
  )

(defclass ik-opt-param
  :super propertied-object
  :slots (*robot-env*
          drive-joint-list
          only-kinematics?
          posture-joint-list
          posture-joint-angle-list
          kin-target-coords-list
          kin-attention-coords-list
          contact-target-coords-list
          contact-attention-coords-list
          contact-constraint-list
          kin-scale-list
          collision-pair-list
          collision-distance-margin
          rarm-wrench
          larm-wrench
          rleg-wrench
          lleg-wrench
          external-wrench-list
          external-coords-list
          pre-process-func
          eom-trans-task-value-thre
          eom-rot-task-value-thre
          kin-task-value-thre
          convergence-check-func
          optimize-torque?
          contact-state
          avoid-collision?
          apply-external-force?

          wait?
          rleg-pos-shift
          lleg-pos-shift
          fix-hand?
          send?
          reset?
          obj
          force-move
          *my-coords*
          *obstacle*
          use-arm
          )
  )

(defmethod ik-opt-param
  (:init (&key  (*my-coords* (instance my-coords :init)))
         (progn
           (setup-jaxon_red-ik-opt :only-kinematics? only-kinematics? );???
           ;;coords
           (send self :set-my-coords *my-coords*)
           ;;ik-params
           (send self :set-contact-state "fix")
           ;;set *robot-env*
           (cond ((equal (send self :get-contact-state) "fix") ;;足は現在位置に固定
                  (setq *robot-env*
                        (instance robot-environment :init :robot *robot*
                                  :root-virtual-joint-min-angle-list (list -1.000000e+07 -1.000000e+07 -1.000000e+07 -1.745329e-08 -1.745329e+08 -1.745329e-08)
                                  :root-virtual-joint-max-angle-list (list 1.000000e+07 1.000000e+07 1.000000e+07 1.745329e-08 1.745329e+08 1.745329e-08))))
                 ((or (equal (send self :get-contact-state) "free") (equal (send self :get-contact-state) "center")) ;;足は設定平面の中心位置or
                  (setq *contact-list*
                        (list (instance 2d-planar-contact :init :name "rleg")
                              (instance 2d-planar-contact :init :name "lleg")))
                  (setq *robot-env*
                        (instance robot-environment :init :robot *robot*
                                  :root-virtual-joint-min-angle-list (list -1.000000e+07 -1.000000e+07 -1.000000e+07 -1.745329e-08 -1.745329e+08 -1.745329e-08)
                                  :root-virtual-joint-max-angle-list (list 1.000000e+07 1.000000e+07 1.000000e+07 1.745329e-08 1.745329e+08 1.745329e-08)
                                  :contact-list *contact-list*))
                  (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
                  (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
                  (send *robot-env* :contact "rleg" :rotate pi :z :local)
                  (send *robot-env* :contact "lleg" :rotate pi :z :local)))

           (send self :set-apply-external-force? t)
           (send self :set-only-kinematics? nil)
           (send self :set-optimize-torque? nil)
           (send self :set-robot-env *robot-env*)
           (send self :set-drive-joint-list
                 (if (and (not (send self :get-only-kinematics?)) (send self :get-optimize-torque?))
                     (send *robot-env* :drive-joint-list) nil))
           (send self :set-posture-joint-list (send *robot-env* :robot :joint-list))
           (send self :set-posture-joint-angle-list (send-all (send *robot-env* :robot :joint-list) :joint-angle))
           (send self :set-kin-target-coords-list
                 (cond ((equal (send self :get-contact-state) "center") ;;足は設定平面の中心位置
                        (list (send *my-coords* :get-ground-rleg)
                              (send *my-coords* :get-ground-lleg)
                              (send *my-coords* :get-rarm-target-coords)
                              (send *my-coords* :get-larm-target-coords)))
                       ((equal (send self :get-contact-state) "free") ;;足は設定平面上自由
                        (list (send *robot-env* :contact "rleg" :contact-coords)
                              (send *robot-env* :contact "lleg" :contact-coords)
                              (send *my-coords* :get-rarm-target-coords)
                              (send *my-coords* :get-larm-target-coords)))
                       ((equal (send self :get-contact-state) "fix") ;;足は現在位置に固定
                        (list (send *my-coords* :get-rleg-coords)
                              (send *my-coords* :get-lleg-coords)
                              (send *my-coords* :get-rarm-coords)
                              (send *my-coords* :get-larm-coords)))))
           (send self :set-kin-attention-coords-list
                 (list (send *robot-env* :robot :get :rleg-contact-coords)
                       (send *robot-env* :robot :get :lleg-contact-coords)
                       (send *robot-env* :robot  :rarm :end-coords)
                       (send *robot-env* :robot  :larm :end-coords)))
           (send self :set-contact-target-coords-list
                 (if (not (send self :get-only-kinematics?))
                     (cond ((equal (send self :get-contact-state) "center")
                            (list (send *my-coords* :get-ground-rleg)
                                  (send *my-coords* :get-ground-lleg)))
                           ((equal (send self :get-contact-state) "free")
                            (list (send *robot-env* :contact "rleg" :contact-coords)
                                  (send *robot-env* :contact "lleg" :contact-coords)))
                           ((equal (send self :get-contact-state) "fix")
                            (list (send *my-coords* :get-rleg-coords)
                                  (send *my-coords* :get-lleg-coords))))))
           (send self :set-contact-attention-coords-list
                 (if (not (send self :get-only-kinematics?))
                     (list (send *robot-env* :robot :get :rleg-contact-coords)
                           (send *robot-env* :robot :get :lleg-contact-coords))))
           (send self :set-contact-constraint-list ;;接触領域の条件を指定（摩擦係数、最大鉛直力、接触領域の長方形？）
                 (if (not (send self :get-only-kinematics?))
                     (list
                      (instance* default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 600
                                 (let* ((poly (send *robot-env* :robot :get :rleg_link5-support-polygon))
                                        (vs   (mapcar #'(lambda (v)
                                                          (send (send (send *robot-env* :robot :get :rleg-contact-coords) :worldcoords)
                                                                :inverse-transform-vector v))
                                                      (send poly :vertices))))
                                   (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                         :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                         :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                         :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20))))
                      (instance* default-contact-constraint
                                 :init
                                 :mu-trans 0.2
                                 :mu-rot 0.01
                                 :max-fz 600
                                 (let* ((poly (send *robot-env* :robot :get :lleg_link5-support-polygon))
                                        (vs   (mapcar #'(lambda (v)
                                                          (send (send (send *robot-env* :robot :get :lleg-contact-coords) :worldcoords)
                                                                :inverse-transform-vector v))
                                                      (send poly :vertices))))
                                   (list :l-min-x (+ (elt (find-extream vs #'(lambda (v) (elt v 0)) #'<) 0) 20)
                                         :l-max-x (- (elt (find-extream vs #'(lambda (v) (elt v 0)) #'>) 0) 20)
                                         :l-min-y (+ (elt (find-extream vs #'(lambda (v) (elt v 1)) #'<) 1) 20)
                                         :l-max-y (- (elt (find-extream vs #'(lambda (v) (elt v 1)) #'>) 1) 20)))))))
           (send self :set-use-arm "both")
           (send self :set-kin-scale-list ;;最適化計算の傾斜 位置、回転6自由度
                 (cond
                  ((equal (send self :get-use-arm) "both") (list (list 1 1 1 1 1 1)
                                            (list 1 1 1 1 1 1)
                                            1.0
                                            1.0))
                  ((equal (send self :get-use-arm) "rarm") (list (list 1 1 1 1 1 1)
                                            (list 1 1 1 1 1 1)
                                            (list 1 1 1 1 0.5 1)
                                            0.0))
                  ((equal (send self :get-use-arm) "larm") (list (list 1 1 1 1 1 1)
                                            (list 1 1 1 1 1 1)
                                            0.0
                                            (list 1 1 1 1 0.5 1)))))
           ;;axis y周りは自由 heaveの時
                            ;; (list 1 1 1 1 0 1)
           ;; (list 1 1 1 1 0 1)
           (send self :set-collision-pair-list
                 (if avoid-collision?
                     (append
                      (if obst? (mapcar #'(lambda (l) (list l *obst*)) (flatten (send *robot-env* :robot :legs :links))))
                      (list (list (send *robot* :link "LLEG_LINK2") (send *robot* :link "LARM_LINK7")))
                      (list (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "LARM_LINK6")))
                      (list (list (send *robot* :link "LLEG_LINK3") (send *robot* :link "LARM_LINK6")))
                      (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "RARM_LINK7")))
                      (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "LLEG_LINK2"))))))
           (send self :set-collision-distance-margin 0.05)
           (send self :set-external-wrench-list
                 (if (and (not (send self :get-only-kinematics?)) (send self :get-apply-external-force?))
                     (list  (float-vector 0 0 530 0 0 0)
                            (float-vector 0 0 530 0 0 0)
                            (float-vector 0 0 -30 0 0 0)
                            (float-vector 0 0 -30 0 0 0))))
           ;;多分絶対座標で与える
           ;; 力の正負が微妙かも
           (send self :set-external-coords-list  ;;外レンチ作用点リスト
                 (if (and (not (send self :get-only-kinematics?)) (send self :get-apply-external-force?))
                     (progn
                       (if (boundp '*robot-env*)
                           (list
                            (send *robot-env* :robot :get :rleg-contact-coords)
                            (send *robot-env* :robot :get :lleg-contact-coords)
                            (send *robot-env* :robot :get :rarm-contact-coords)
                            (send *robot-env* :robot :get :larm-contact-coords))
                         (list
                          (send *robot* :rleg :end-coords)
                          (send *robot* :lleg :end-coords)
                          (send *robot* :rarm :end-coords)
                          (send *robot* :larm :end-coords))))))
                        ;; (list (send *robot-env* :robot :get :rarm-contact-coords)
           ;;       (send *robot-env* :robot :get :larm-contact-coords))

           ;;optimization
           (send self :set-pre-process-func pre-process-func)
           (send self :set-eom-rot-task-value-thre 0.01)
           ;; (send self :set-eom-trans-task-value-thre 0.01)
           (send self :set-eom-trans-task-value-thre 10)
           (send self :set-kin-task-value-thre 0.05)
           (send self :set-convergence-check-func
                 #'(lambda(sqp)
                     (if (and (< (norm (send (send sqp :config-task) :kinematics-task-value :update? nil)) (send self :get-kin-task-value-thre))
                              (< (norm (send (send sqp :config-task) :eom-rot-task-value :update? nil)) (send self :get-eom-rot-task-value-thre))
                              (< (norm (send (send sqp :config-task) :eom-trans-task-value :update? nil)) (send self :get-eom-trans-task-value-thre)))
                         t ;converge
                       nil)))
           ;; (send self :set-optimize-torque? nil)
           (send self :set-contact-state nil)
           (send self :set-avoid-collision? t)
           (send self :set-obstacle (make-cube 80 1000 500))

           ;;不要？
           (send self :set-rleg-pos-shift #f(80 -20 0))
           (send self :set-lleg-pos-shift #f(80 20 0))
           (send self :set-fix-hand? nil)
           (send self :set-send? t)
           (send self :set-reset? t)
           (send self :set-force-move     nil)
           )
         self)
  ;;set
  (:set-robot-env (val) (setq *robot-env* val))
  (:set-drive-joint-list (val) (setq drive-joint-list val))
  (:set-only-kinematics? (val)                  (setq only-kinematics? val))
  (:set-apply-external-force? (val)             (setq apply-external-force? val))
  (:set-posture-joint-list (val)                (setq posture-joint-list val))
  (:set-posture-joint-angle-list (val)          (setq posture-joint-angle-list val))
  (:set-kin-target-coords-list (val)            (setq kin-target-coords-list val))
  (:set-kin-attention-coords-list (val)         (setq kin-attention-coords-list val))
  (:set-contact-target-coords-list (val)        (setq contact-target-coords-list val))
  (:set-contact-attention-coords-list (val)     (setq contact-attention-coords-list val))
  (:set-contact-constraint-list (val)               (setq contact-constraint-list val))
  (:set-use-arm (val)                           (setq use-arm val))
  (:set-kin-scale-list (val)                    (setq kin-scale-list val))
  (:set-collision-pair-list (val)               (setq collision-pair-list val))
  (:set-collision-distance-margin (val)         (setq collision-distance-margin val))
  (:set-external-wrench-list(val)                (setq external-wrench-list val))
  (:set-external-coords-list(val)               (setq external-coords-list val))
  (:set-pre-process-func (val)                  (setq pre-process-func val))
  (:set-eom-trans-task-value-thre (val)         (setq eom-trans-task-value-thre  val))
  (:set-eom-rot-task-value-thre (val)           (setq eom-rot-task-value-thre val))
  (:set-kin-task-value-thre (val)               (setq kin-task-value-thre  val))
  (:set-convergence-check-func (val)            (setq convergence-check-func val))
  (:set-optimize-torque? (val)                  (setq optimize-torque? val))
  (:set-contact-state (val)                      (setq contact-state val))
  (:set-avoid-collision? (val)                  (setq avoid-collision? val))
  (:set-my-coords (val)                         (setq *my-coords* val))
  (:set-obstacle (val)                          (setq *obstacle* val))

  ;;不要？
  (:set-wait? (val)                             (setq wait? val))
  (:set-rleg-pos-shift(val)                     (setq rleg-pos-shift val))
  (:set-lleg-pos-shift(val)                     (setq lleg-pos-shift val))
  (:set-fix-hand?(val)                          (setq fix-hand? val))
  (:set-send?(val)                              (setq send? val))
  (:set-reset?(val)                             (setq reset? val))
  (:set-object(val)                             (setq obj val))
  (:set-force-move(val)                         (setq force-move val))

  ;;get
  (:get-robot-env() *robot-env*)
  (:get-drive-joint-list () drive-joint-list)
  (:get-only-kinematics? ()                      only-kinematics?)
  (:get-apply-external-force? ()                 apply-external-force?)
  (:get-posture-joint-list () posture-joint-list)
  (:get-posture-joint-angle-list () posture-joint-angle-list)
  (:get-kin-target-coords-list ()            kin-target-coords-list)
  (:get-kin-attention-coords-list ()          kin-attention-coords-list)
  (:get-contact-target-coords-list ()          contact-target-coords-list )
  (:get-contact-attention-coords-list ()       contact-attention-coords-list )
  (:get-contact-constraint-list ()                 contact-constraint-list )
  (:get-use-arm ()                           use-arm)
  (:get-kin-scale-list ()                        kin-scale-list)
  (:get-collision-pair-list ()                 collision-pair-list )
  (:get-collision-distance-margin ()         collision-distance-margin )
  (:get-external-wrench-list()                    external-wrench-list )
  (:get-external-coords-list()                   external-coords-list)
  (:get-pre-process-func ()                 pre-process-func)
  (:get-eom-trans-task-value-thre ()             eom-trans-task-value-thre)
  (:get-eom-rot-task-value-thre ()               eom-rot-task-value-thre)
  (:get-kin-task-value-thre ()                   kin-task-value-thre)
  (:get-convergence-check-func()             convergence-check-func )
  (:get-optimize-torque? ()                      optimize-torque?)
  (:get-contact-state ()                          contact-state )
  (:get-avoid-collision? ()                      avoid-collision? )
  (:get-my-coords ()                         *my-coords*)
  (:get-obstacle ()                          *obstacle*)

  ;;不要？
  (:get-wait? ()                                 wait? )
  (:get-rleg-pos-shift()                         rleg-pos-shift )
  (:get-lleg-pos-shift()                         lleg-pos-shift )
  (:get-fix-hand?()                              fix-hand? )
  (:get-send?()                                  send? )
  (:get-reset?()                                 reset? )
  (:get-object()                                 obj)
  (:get-force-move()                          force-move )
  )

(defclass my-ik-param
  :super propertied-object
  :slots (only-kinematics? optimize-torque? fix-contact? avoid-collision? apply-external-force? wait? rleg-pos-shift lleg-pos-shift external-wrench-list set-unique-kin-scale? fix-hand?  send? reset? kin-scale-list external-coords-list rleg-force-offset lleg-force-offset kin-task-value-thre eom-trans-task-value-thre eom-rot-task-value-thre obj force-move)
  )

(defmethod my-ik-param
  (:init ()
         (progn
           (send self :set-only-kinematics? nil)
           (send self :set-optimize-torque? nil)
           (send self :set-fix-contact? nil)
           (send self :set-avoid-collision? t)
           (send self :set-apply-external-force? t)
           (send self :set-wait? t)
           (send self :set-rleg-pos-shift #f(80 -20 0))
           (send self :set-lleg-pos-shift #f(80 20 0))
           ;; (send self :set-external-wrench-list (list
           ;;                                      (float-vector 0 0 400 0 0 0)
           ;;                                      (float-vector 0 0 400 0 0 0)
           ;;                                      (float-vector 0 0 -80 0 0 0)
           ;;                                      (float-vector 0 0 -80 0 0 0)))
           (send self :set-rarm-wrench (float-vector 0 0 -30 0 0 0))
           (send self :set-larm-wrench (float-vector 0 0 -30 0 0 0))
           (send self :set-rleg-wrench-offset (float-vector 0 0 530 0 0 0))
           (send self :set-lleg-wrench-offset (float-vector 0 0 530 0 0 0))
           ;; (send self :set-rleg-wrench-offset (float-vector 0 0 0 0 0 0))
           ;; (send self :set-lleg-wrench-offset (float-vector 0 0 0 0 0 0))

           (send self :set-rleg-wrench rleg-wrench-offset)
           (send self :set-lleg-wrench lleg-wrench-offset)
           (send self :set-eom-rot-task-value-thre 0.01)
           ;; (send self :set-eom-trans-task-value-thre 0.01)
	   (send self :set-eom-trans-task-value-thre 10)
           (send self :set-kin-task-value-thre 0.005)
           (send self :set-set-unique-kin-scale? nil)
           (send self :set-fix-hand? nil)
           (send self :set-send? t)
           (send self :set-reset? t)
           (send self :set-kin-scale-list (list 1.0
                                                1.0
                                                (list 1 1 1 1 1 0.2)
                                                0.0))
           (send self :set-external-coords-list (if (boundp '*robot-env*)
                                                    (list
                                                     (send *robot-env* :robot :get :rleg-contact-coords)
                                                     (send *robot-env* :robot :get :lleg-contact-coords)
                                                     (send *robot-env* :robot :get :rarm-contact-coords)
                                                     (send *robot-env* :robot :get :larm-contact-coords))
                                                  (list
                                                   (send *robot* :rleg :end-coords)
                                                   (send *robot* :lleg :end-coords)
                                                   (send *robot* :rarm :end-coords)
                                                   (send *robot* :larm :end-coords))))
	   (send self :set-force-move     nil)
           )
         self)


  (:set-only-kinematics? (val)                  (setq only-kinematics? val))
  (:set-optimize-torque? (val)                  (setq optimize-torque? val))
  (:set-fix-contact? (val)                      (setq fix-contact? val))
  (:set-avoid-collision? (val)                  (setq avoid-collision? val))
  (:set-apply-external-force? (val)             (setq apply-external-force? val))
  (:set-wait? (val)                             (setq wait? val))
  (:set-rleg-pos-shift(val)                     (setq rleg-pos-shift val))
  (:set-lleg-pos-shift(val)                     (setq lleg-pos-shift val))
  (:set-external-wrench-list(val)                (setq external-wrench-list val))
  (:set-set-unique-kin-scale?(val)              (setq set-unique-kin-scale? val))
  (:set-fix-contact?(val)                       (setq fix-contact? val))
  (:set-fix-hand?(val)                          (setq fix-hand? val))
  (:set-send?(val)                              (setq send? val))
  (:set-reset?(val)                             (setq reset? val))
  (:set-kin-scale-list (val)                    (setq kin-scale-list val))
  (:set-external-coords-list(val)               (setq external-coords-list val))
  (:set-rarm-wrench(val)                         (setq rarm-wrench val))
  (:set-larm-wrench(val)                         (setq larm-wrench val))
  (:set-rleg-wrench(val)                         (setq lleg-wrench val))
  (:set-lleg-wrench(val)                         (setq rleg-wrench val))
  (:set-rleg-wrench-offset(val)                  (setq rleg-wrench-offset val))
  (:set-lleg-wrench-offset(val)                  (setq lleg-wrench-offset val))
  (:set-kin-task-value-thre (val)               (setq kin-task-value-thre  val))
  (:set-eom-trans-task-value-thre (val)         (setq eom-trans-task-value-thre  val))
  (:set-eom-rot-task-value-thre (val)           (setq eom-rot-task-value-thre val))
  (:set-object(val)                             (setq obj val))
  (:set-force-move(val)                         (setq force-move val))

  (:get-only-kinematics? ()                      only-kinematics?)
  (:get-optimize-torque? ()                      optimize-torque?)
  (:get-fix-contact? ()                          fix-contact? )
  (:get-avoid-collision? ()                      avoid-collision? )
  (:get-apply-external-force? ()                 apply-external-force? )
  (:get-wait? ()                                 wait? )
  (:get-rleg-pos-shift()                         rleg-pos-shift )
  (:get-lleg-pos-shift()                         lleg-pos-shift )
  (:get-external-wrench-list()                    external-wrench-list )
  (:get-set-unique-kin-scale?()                  set-unique-kin-scale? )
  (:get-fix-contact?()                           fix-contact? )
  (:get-fix-hand?()                              fix-hand? )
  (:get-send?()                                  send? )
  (:get-reset?()                                 reset? )
  (:get-kin-scale-list ()                        kin-scale-list)
  (:get-external-coords-list()                   external-coords-list)
  (:get-rarm-wrench()                             rarm-wrench)
  (:get-larm-wrench()                             larm-wrench)
  (:get-lleg-wrench()                             lleg-wrench)
  (:get-rleg-wrench()                             rleg-wrench)
  (:get-rleg-wrench-offset()                      rleg-wrench-offset)
  (:get-lleg-wrench-offset()                      lleg-wrench-offset)
  (:get-eom-trans-task-value-thre ()             eom-trans-task-value-thre)
  (:get-eom-rot-task-value-thre ()               eom-rot-task-value-thre)
  (:get-kin-task-value-thre ()                   kin-task-value-thre)
  (:get-object()                                 obj)
  (:get-force-move(val)                          force-move )
  )

(defclass poses
  :super propertied-object
  :slots (fin-pose-ri fin-pose-robot )
  )
(defmethod poses
  (:init (pose-robot pose-ri)
         (send self :set-fin-pose-robot pose-robot)
         (send self :set-fin-pose-ri pose-ri)
         self
         )
  (:set-fin-pose-ri (val)
                    (setq fin-pose-ri val))
  (:set-fin-pose-robot (val)
                 (setq fin-pose-robot val))
  (:get-fin-pose-ri ()
                    fin-pose-ri )
  (:get-fin-pose-robot ()
                    fin-pose-robot )

  )

(defclass point-num-save
  :super propertied-object
  :slots (larm-points-num rarm-points-num)
  )
(defmethod point-num-save
  (:init (larm-points-num-init rarm-points-num-init)
         (send self :set-larm-points-num larm-points-num-init)
         (send self :set-rarm-points-num rarm-points-num-init)
         self
         )
  (:set-larm-points-num (val)
                        (setq larm-points-num val))
  (:set-rarm-points-num (val)
                    (setq rarm-points-num val))
  )

;; (setq *poses* (instance poses :init
;;                        (send *robot* :angle-vector)
;;                        (send *ri* :state :angle-vector)))
;; (setq *coords* (instance my-coords :init
;;                        (copy-object (send *robot* :rarm :end-coords :copy-worldcoords))
;;                        (copy-object (send *robot* :larm :end-coords :copy-worldcoords))
;;                        (copy-object (send *robot* :rleg :end-coords :copy-worldcoords))
;;                        (copy-object (send *robot* :lleg :end-coords :copy-worldcoords))
;;                        #f(0 0 700)
;;                        ))

;; (setq *poses* (instance poses :init
;;                        (send *robot* :angle-vector)
;;                        (send *ri* :state :angle-vector)))
