
(defun average
    (lis &key  (axis 10))
  (let* ((len (list-length lis))
         ;; (sum (list 0 0 0)))
         (sum_x 0)
         (sum_y 0)
         (sum_z 0))
         
    (dotimes (i len)
      (setq sum_x (+ sum_x (elt (elt lis i) 0)))
      (setq sum_y (+ sum_y (elt (elt lis i) 1)))
      (setq sum_z (+ sum_z (elt (elt lis i) 2)))      
      )
        
    (setq ave (float-vector (/ sum_x (float len)) (/ sum_y (float len)) (/ sum_z (float len))))

    ;; (print (format nil "ave = ~A" ave))
    ;; (print len)
    (cond
     ((equal axis 0)
      (float (elt ave 0)))
     ((equal axis 1)
      (elt ave 1))
     ((equal axis 2)
      (float (elt ave 2)))
     (t (progn ;;(print "output vector")
               ave))
     )
    )
  )

;;axis 0 1 2
(defun quick-sort-vector-list(seq &key(axis 0))
  ;; (format t "axis =~A~%" axis)
  (if (< (length seq) 2)
      (return-from quick-sort-vector-list seq :axis axis))
  (let ((pivot (elt (elt seq 0) axis))
        (pivot-vec (elt seq 0) )
        (left nil)
        (right nil))

  (dotimes (i (- (length seq) 1))
    (if (< (elt (elt seq (+ i 1)) axis) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort-vector-list left :axis axis))
  (setq right (quick-sort-vector-list right :axis axis))
  (setq fwd (append left (list pivot-vec)))
  (setq ret (append fwd right ))
  ret
  ))


;;listのうちx要素が小さいものをnumこ取り出す
(defun get-min-val(lis num &key(axis 0))
  (setq ret-lis (quick-sort-vector-list lis :axis axis))
  (subseq ret-lis 0 num))


(defun quick-sort(seq)
    (if (< (length seq) 1)
      (return-from quick-sort seq))
  (let ((pivot (elt seq 0))
        ( left nil)
        ( right nil))

  (dotimes (i (- (length seq) 1))
    ;; (print i)
    (if (< (elt seq (+ i 1)) pivot)
        (setq left (list-insert (elt seq (+ i 1)) (length seq) left))
      (setq right (list-insert (elt seq (+ i 1)) (length seq) right))
      ))
  ;; (format t "left = ~A~%" left)
  ;; (format t "right = ~A~%" right)
  (setq left (quick-sort left))
  (setq right (quick-sort right))
  (setq fwd (append left (list pivot) ))
  (setq ret (append fwd right ))
  ret
  ))

(print "functions-without-ri.l")
