(require "models/arrow-object.l")
(require "package://control_tools/log-tools.l")
(require "package://control_tools/play-hrpsysfile.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(require "package://rm_debris_vision/euslisp/manip_tire/my-ik-latest.l")
(require "package://rm_debris_vision/euslisp/manip_tire/variables.l")
(require "package://rm_debris_vision/euslisp/manip_tire/params.l")

(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(defun manip-tire()
  (let* ((*my-coords* (instance my-coords :init)))
    (send *my-coords* :set-rarm-coords (make-coords :pos (float-vector 300 -400 100) :rpy (float-vector 0 (deg2rad  60) (deg2rad -90))))
    (send *my-coords* :set-larm-coords (make-coords :pos (float-vector 300  400 100) :rpy (float-vector 0 (deg2rad  60) (deg2rad  90))))

    (setq *ik-opt-param* (instance ik-opt-param :init :*my-coords* *my-coords*))
    (ik-opt-solver :*ik-opt-param* *ik-opt-param*)
         ))
