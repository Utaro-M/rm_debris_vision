#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "check_in_hand_server")]

(setq flag-pcl 0)
(setq flag 0)
(checking-time 6)
(sleep-time 20000)
(working? t)
(thre-list (list 10 10))

;;output flag   0 ::both arms holding , 1 ::rarm drop , 2 ::larm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-force-pcl
    (&key (checking-time 6) (sleep-time 20000) (working? t) (thre-list (list 10 10)) (fin 3))
  (setq initial (send *robot* :angle-vector)
        rarm-f-pre  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil)
        larm-f-pre  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil)
        angle-pre (send *ri* :state :angle-vector))


  (if working? 
      (format t "~%~%check while working~%")
    (format t "~%~%check while still~%"))
  
  (dotimes (i (/ (* checking-time 1000000) sleep-time))
    (setq flag 0)
    ;; (format t "flag = ~A~%" flag)
    (setq rarm-f  (low-pass-online "force" "absolute" :rarm :times 10 :sleep 100 :flag nil))
    (setq larm-f  (low-pass-online "force" "absolute" :larm :times 10 :sleep 100 :flag nil))
    ;;絶対値バージョン
    (setq rarm-f-dif (abs (elt (v- rarm-f rarm-f-pre) 2)))
    (setq larm-f-dif (abs (elt (v- larm-f larm-f-pre) 2)))
    ;; (setq rarm-f-dif (elt (v- rarm-f rarm-f-pre) 2))
    ;; (setq larm-f-dif (elt (v- larm-f larm-f-pre) 2))
    (setq drop (list nil nil))
    (format t "rarm-f-dif       = ~A~%" rarm-f-dif)
    (format t "larm-f-dif       = ~A~%" larm-f-dif)

    ;;drop判定 ;;へんこうする
    (if (>  larm-f-dif (* +1 (elt thre-list 1)))
        (print "larm dif")
        (setq flag (+ flag 1)))
    (if (>  rarm-f-dif (* +1 (elt thre-list 0)))
        (setq flag (+ flag 2)))
    (format t "flag = ~A~%" flag)
    (setq flag-pcl (check-pcl))
    (format t "flags = ~A ~A ~%" flag flag-pcl)
    (if (= flag flag-pcl)
        (if (= flag 0)
            (print "holding")
          (progn
            (cond 
             ((= flag 1)
              (print "drop larm"))
             ((= flag 2)
              (print "drop rarm"))
             ((= flag 3)
              (print "drop both arm")))
            (return-from check-force-pcl flag))))
      
    (if working?
        (progn
          ;;動作終了判定
          (setq tmp (send *ri* :reference-vector))
          (setq angle-vector-dif (abs (norm (v- initial tmp))))
          (format t "angle-vector-dif = ~A~%" angle-vector-dif)
          (if (< angle-vector-dif 0.5)
              (progn
                (print "finish")
                (setq flag 10)
                (return angle-vector-dif))
            )
          (setq rarm-f-pre rarm-f)
          (setq larm-f-pre larm-f)))
    
    (setq angle-pre (send *ri* :state :angle-vector))
    
    (unix:usleep sleep-time)
    )
  (format t "flag = ~A~%" flag)
  flag
  )
(setq drop_flag_larm (instance std_msgs::Bool :init))
(setq drop_flag_rarm (instance std_msgs::Bool :init))
(send drop_flag_larm :data nil)
(send drop_flag_rarm :data nil)
;;output flag   0 ::both arms holding , 1 ::rarm drop , 2 ::larm drop , 3 ::both arms drop, 10 ::finish smoothly
(defun check-pcl()
  (let*
      ((drop_flag_larm (one-shot-subscribe "/drop_flag_larm" std_msgs::Bool :timeout 3000))
       (drop_flag_rarm (one-shot-subscribe "/drop_flag_rarm" std_msgs::Bool :timeout 3000))
       (flag 0))
    (format t "get larm points = ~A~%" (send drop_flag_larm :data))
    (format t "get rarm points = ~A~%" (send drop_flag_rarm :data))
    (if (not (send drop_flag_larm :data))
        (setq flag 2))
    (if (not (send drop_flag_rarm :data))
        (setq flag 1))
    (if (and (not (send drop_flag_larm :data)) (not (send drop_flag_rarm :data)))
        (setq flag 3))
    flag)
  )

(defun send_drop_flag()
  (let ((m (send req :response)))
    (send m :pts points)
    m
  ))
(ros::advertise-service "check" rm_debris_vision::Check #'send_plane)
(do-until-key
 (check-force-pcl)
 (ros::spin-once)
 )

