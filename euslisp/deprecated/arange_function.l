;;;整理中に板持ち上げのときのik-opt()を整理後のsolve-ik-optを使ったバージョンに変えようとした痕跡2022/01/14

(defun leg-free
    (avoid-collision?
     fix-contact?)

  (setq *ik-opt-param* (instance ik-opt-param))

  (when avoid-collision?
    ;;big board version
    (format t "avoid collision~%")
    (setq *obst* (make-cube 80 1000 500))
    (if heave?
        (send *obst* :translate (v+ (scale 0.5 (v+ (send rarm-coords :worldpos) (send larm-coords :worldpos))) #f(-30 0 -400)))
      (send *obst* :translate (v+ (scale 0.5 (v+ (send rarm-coords :worldpos) (send larm-coords :worldpos))) #f(0 0 -400))))
    (send *obst* :set-color (float-vector 0 1 1) 0.5)
    )
  
  ;; setup robot
  (setup-jaxon_red-ik-opt :only-kinematics? only-kinematics? )
  ;;robot-env
  (cond
   (fix-contact?
    (setq *robot-env*
          (instance robot-environment :init :robot *robot*
                    :root-virtual-joint-min-angle-list (list -1.000000e+07 -1.000000e+07 -1.000000e+07 -1.745329e-08 -1.745329e+08 -1.745329e-08)
                    :root-virtual-joint-max-angle-list (list 1.000000e+07 1.000000e+07 1.000000e+07 1.745329e-08 1.745329e+08 1.745329e-08))))

   (t
    (setq *contact-list*
          (list (instance 2d-planar-contact :init :name "rleg")
                (instance 2d-planar-contact :init :name "lleg")))
    (setq *robot-env*
          (instance robot-environment :init :robot *robot*
                    :root-virtual-joint-min-angle-list (list -1.000000e+07 -1.000000e+07 -1.000000e+07 -1.745329e-08 -1.745329e+08 -1.745329e-08)
                    :root-virtual-joint-max-angle-list (list 1.000000e+07 1.000000e+07 1.000000e+07 1.745329e-08 1.745329e+08 1.745329e-08) :contact-list *contact-list*))
    (send *robot-env* :contact "rleg" :set-from-face :face (elt (send *ground-rleg* :faces) 1))
    (send *robot-env* :contact "lleg" :set-from-face :face (elt (send *ground-lleg* :faces) 1))
    (send *robot-env* :contact "rleg" :rotate pi :z :local)
    (send *robot-env* :contact "lleg" :rotate pi :z :local)))

  (if reset? (send *robot-env* :robot :reset-manip-pose))
  
  (send *ik-opt-param* :set-robot-env *robot-env*)

  ;; draw
  (unless (boundp '*irtviewer*)
    (make-irtviewer)
    (send *irtviewer* :change-background (float-vector 1 1 1))
    (send *irtviewer* :draw-floor 500)
    (send *irtviewer* :floor-color (float-vector 0 0 0))
    )

    ;;04 make picture for thesis
   (progn (setq bxr (make-cube 200 200 250 )) (setq bxl (make-cube 200 200 250 )) (send bxr :set-color #f(0 0 1 0.5)) (send bxl :set-color #f(0 0 1 0.5))  (send bxr :newcoords (make-coords :coords (send *robot* :rarm :end-coords :copy-worldcoords))) (send bxl :newcoords (make-coords :coords (send *robot* :larm :end-coords :copy-worldcoords))) )
   (objects (append (list  *robot*)))

   (solve-ik-opt :params *ik-opt-param*)
   
   (send (send *robot* :joint "RLEG_JOINT3") :min-angle 0)
   (send (send *robot* :joint "LLEG_JOINT3") :min-angle 0)
   (if (equal (send *sqp-opt* :status) :not-converged)
       (progn
         (format t "ik failed~%")
         (send *robot* :angle-vector (send *coords* :get-pre-angle-vector))
         (send *robot* :move-coords (send *coords* :get-lleg-coords) (send *robot* :lleg :end-coords))
         nil)
     t)
   )
