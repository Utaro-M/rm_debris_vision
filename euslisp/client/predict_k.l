#!/usr/bin/env roseus
(ros::load-ros-manifest "rm_debris_vision")
(ros::roseus "get_pred_k")
(ros::wait-for-service "pred_k")
(load "../functions-without-ri.l")

;;#2f((0.0 0.0 0.0) (0.0 0.0 0.0) (0.0 0.0 0.0))型のマトリクスをうけとり、その固有値最少の固有ベクトルを返す
;; 固有値が虚数になる場合は、元の動かした方向のうちよく動いたベクトルを返す
(defun pred-k(&key (mat) (wrench-mat) (pose-mat) (use-force? t))
  (let ((req (instance rm_debris_vision::PredKRequest :init)))
    ;; (setq eign-eignvector-list (pred-k (list (matrix-row k-mat 0) (matrix-row k-mat 1) (matrix-row k-mat 2)))) ;;大きい順
    ;; (format t "req =~A~%" req)
    (if use-force?
        (progn
          (format t "mat=~A~%" mat)
          (setq mat-service (list (instance geometry_msgs::Point32 :init) (instance geometry_msgs::Point32 :init) (instance geometry_msgs::Point32 :init)))
          (dotimes (i 3)
            (progn
              (send (elt mat-service i) :x (aref mat i 0))
              (send (elt mat-service i) :y (aref mat i 1))
              (send (elt mat-service i) :z (aref mat i 2))
              ))
          (send req :mat :points mat-service)
          
          (setq res (ros::service-call "pred_k" req t))
          (format t "flag=~A~%" (send res :flag :data))
          (if (send res :flag :data)
              (progn
                (print "success to get eign")
                ;;serviceを使うようにした
                ;; (setq eign-sub (one-shot-subscribe "/pred_k_node/eign" std_msgs::Float64MultiArray :timeout 2000))
                ;; (setq eign (send eign-sub :data))
                ;; (format t "eign =~A~%" eign)
                ;; (setq eignvector-sub (one-shot-subscribe "/pred_k_node/eign_vector" geometry_msgs::Polygon :timeout 2000))
                ;; ;; (setq eign (send eign-sub :data))
                ;; (setq eignvector (mapcar #'(lambda (vertice) (float-vector (send vertice :x) (send vertice :y) (send vertice :z))) (send eignvector-sub :points)))
                ;; (format t "eignvector =~A~%" eignvector)
                ;;service version
                (setq eign-service (send res :eign :data))
                (setq eignvector-service (mapcar #'(lambda (vertice) (float-vector (send vertice :x) (send vertice :y) (send vertice :z))) (send res :eignvector :points)))
                (format t "eign-service =~A~%" eign-service)
                (format t "eignvector-service =~A~%" eignvector-service)
                (setq eign-abs-list (list (abs (elt eign-service 0)) (abs (elt eign-service 1)) (abs (elt eign-service 2))))
                (setq sorted-eign-abs-list (quick-sort eign-abs-list))
                (format t "sorted-eign-abs-list=~A~%" sorted-eign-abs-list)
                (setq min-idx (- (length eign-abs-list) (length (memq (car sorted-eign-abs-list) eign-abs-list))))
                (format t "min-idx =~A~%" min-idx)
                ;; (setq eign-abs-list (list (/ (elt eign-service 0) (abs (elt eign-service 0))) (/ (elt eign-service 1) (abs (elt eign-service 1))) (/ (elt eign-service 2) (abs (elt eign-service 2))))) ;;1 or -1
                (setq movable-vector (elt eignvector-service min-idx))) ;;smallest
            (progn
              (print "fail to get eign")
              ;; f/x=kが小さい方向を見つける
              (setq f-x-ratio (list))
              (dotimes (i (array-dimension wrench-mat 1))
                ;;先頭に追加
                (push (/ (norm (float-vector (aref wrench-mat 0 i) (aref wrench-mat 1 i) (aref wrench-mat 2 i))) (norm (float-vector (aref pose-mat 0 i) (aref pose-mat 1 i) (aref pose-mat 2 i)))) f-x-ratio))
              (setq f-x-ratio (reverse f-x-ratio))
              (format t "f-x-ratio=~A~%" f-x-ratio)
              (setq sorted-f-x-ratio (quick-sort f-x-ratio)) ;;小さい順
              (format t "sorted-f-x-ratio=~A~%" sorted-f-x-ratio)
              (setq min-idx (- (array-dimension pose-mat 1) (length (memq (car sorted-f-x-ratio) f-x-ratio)) ))
              ;; (setq min-idx (memq (car sorted-f-x-ratio) f-x-ratio))
              (format t "min-idx =~A~%" min-idx)
              (setq movable-vector (float-vector (aref pose-mat 0 min-idx) (aref pose-mat 1 min-idx) (aref pose-mat 2 min-idx)))
              )))
      
      (progn
        (print "do not use force")
        ;; pos-diffが小さい方向を見つける
        (format t "pose-mat=~A~%" pose-mat)
        (setq pos-diff (list))
        (dotimes (i (array-dimension pose-mat 1))
          ;;先頭に追加
          (push  (norm (float-vector (aref pose-mat 0 i) (aref pose-mat 1 i) (aref pose-mat 2 i))) pos-diff))
        (setq pos-diff (reverse pos-diff))
        (format t "pos-diff=~A~%" pos-diff)
        (setq sorted-pos-diff (quick-sort pos-diff)) ;;小さい順
        (setq sorted-pos-diff (reverse sorted-pos-diff))
        (format t "sorted-pos-diff=~A~%" sorted-pos-diff)
        (setq max-idx (- (array-dimension pose-mat 1) (length (memq (car sorted-pos-diff) pos-diff)) ))
        (format t "max-idx =~A~%" max-idx)
        (setq movable-vector (float-vector (aref pose-mat 0 max-idx) (aref pose-mat 1 max-idx) (aref pose-mat 2 max-idx))))
      )
    (setq movable-vector (normalize-vector movable-vector))
    (format t "k movable vector = ~A~%" movable-vector)
    movable-vector
    ))

;; (defun v-items(vec1 vec2)
;;   (setq lis (make-list 0))
;;   (dotimes (i (length vec1))
;;     (push (* (elt vec1 i) (elt vec2 i)) lis)
;;     )
;;   (mapcar #'(lambda (vertice) (float-vector )) (reverse lis))
