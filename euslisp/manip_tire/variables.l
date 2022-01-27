(require "package://rm_debris_vision/euslisp/manip_tire/params.l")

(defun set-valiable
    (&key
     (mode "tyre")
     (*my-coords* (instance my-coords)))

  (setq *ik-opt-param* (instance ik-opt-param *my-coords*))
  (format t "set ik-opt-params~%")

  (cond ((equal mode "tyre")
         (progn;;default
           (format t "tyre mode~%")
           ))
        (t
         (format t "no such mode~%")))

  *ik-opt-param*
  )


