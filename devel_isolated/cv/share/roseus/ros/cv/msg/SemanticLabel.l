;; Auto-generated. Do not edit!


(when (boundp 'cv::SemanticLabel)
  (if (not (find-package "CV"))
    (make-package "CV"))
  (shadow 'SemanticLabel (find-package "CV")))
(unless (find-package "CV::SEMANTICLABEL")
  (make-package "CV::SEMANTICLABEL"))

(in-package "ROS")
;;//! \htmlinclude SemanticLabel.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass cv::SemanticLabel
  :super ros::object
  :slots (_label _point ))

(defmethod cv::SemanticLabel
  (:init
   (&key
    ((:label __label) "")
    ((:point __point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _point __point)
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(setf (get cv::SemanticLabel :md5sum-) "e5c742a84cd4771bb6f0e4d32a54b224")
(setf (get cv::SemanticLabel :datatype-) "cv/SemanticLabel")
(setf (get cv::SemanticLabel :definition-)
      "string label
geometry_msgs/Point point
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :cv/SemanticLabel "e5c742a84cd4771bb6f0e4d32a54b224")


