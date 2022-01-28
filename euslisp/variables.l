(require "package://rm_debris_vision/euslisp/params.l")

(defun set-valiable
    (&key
     (mode "tire")
     (*my-coords* (instance my-coords)))

  (setq *ik-opt-param* (instance ik-opt-param :init :*my-coords* *my-coords*))
  (format t "set ik-opt-params~%")

  (cond ((equal mode "tire")
         (progn;;default
           (setq obstacle (make-cube 400 400 200))
           (send obstacle :newcoords (make-coords :pos (float-vector (+ 400 (elt (send (send *robot* :rleg :end-coords) :worldpos) 0)) 0 100)))
           (send *ik-opt-param* :set-obstacle obstacle)

           (send *ik-opt-param* :set-collision-pair-list
                     ;; (append
                      (mapcar #'(lambda (l) (list l obstacle)) (flatten (send (send *ik-opt-param* :get-robot-env) :robot :legs :links))))
                      ;; (list (list (send *robot* :link "LLEG_LINK2") (send *robot* :link "LARM_LINK7")))
                      ;; (list (list (send *robot* :link "CHEST_LINK1") (send *robot* :link "LARM_LINK6")))
                      ;; (list (list (send *robot* :link "LLEG_LINK3") (send *robot* :link "LARM_LINK6")))
                      ;; (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "RARM_LINK7")))
                      ;; (list (list (send *robot* :link "RLEG_LINK2") (send *robot* :link "LLEG_LINK2"))))
           (format t "tire mode~%")
           ))
        (t
         (format t "no such mode~%")))
  *ik-opt-param*
  )


