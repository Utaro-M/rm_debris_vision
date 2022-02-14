(require "models/arrow-object.l")
;; (require "package://control_tools/log-tools.l")
;; (require "package://control_tools/play-hrpsysfile.l")
(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")

(require "package://rm_debris_vision/euslisp/my-ik-latest.l")
(require "package://rm_debris_vision/euslisp/variables.l")
(require "package://rm_debris_vision/euslisp/params.l")
(require "package://rm_debris_vision/euslisp/client/change_clip_area_client.l")

(unless (and (boundp '*robot*) (boundp '*ri*))
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(defun manip-tire(&key (send? nil) (pose-idx 0))
  (let* ((*my-coords* (instance my-coords :init))
         (targetpos-list '((#f(450 -450 550) #f(450  450 550)) (#f(450 -450 150) #f(450  450 150)) (#f(450 -480 150) #f(450  480 150)))))

    (send-clip-box :target "in_hand" :box-position-list  '(#f(0 0 100) #f(0 0 100))  :size-vector-list (list #f(0.2 0.2 0.1) #f(0.2 0.2 0.1)) :tilt-vector-list (list #f(0 0 0) #f(0 0 0)))
    (format t "(elt (elt targetpos-list pose-idx) 0) = ~A~%"  (elt (elt targetpos-list pose-idx) 0))
    (send *my-coords* :set-rarm-coords (make-coords :pos (elt (elt targetpos-list pose-idx) 0) :rpy (float-vector 0 (deg2rad  60) (deg2rad -90))))
    (send *my-coords* :set-larm-coords (make-coords :pos (elt (elt targetpos-list pose-idx) 1) :rpy (float-vector 0 (deg2rad  60) (deg2rad  90))))

    (setq *ik-opt-param* (set-valiable :mode "tire" :*my-coords* *my-coords*));; (instance ik-opt-param :init :*my-coords* *my-coords*))
    (ik-opt-solver :*ik-opt-param* *ik-opt-param*)
    (if send?
        (send *ri* :angle-vector (send *robot* :angle-vector) 2000))))

;; rosrun rviz rviz -d manip_tire.rviz
