; Auto-generated. Do not edit!


(cl:in-package cv-msg)


;//! \htmlinclude SemanticLabel.msg.html

(cl:defclass <SemanticLabel> (roslisp-msg-protocol:ros-message)
  ((label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass SemanticLabel (<SemanticLabel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SemanticLabel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SemanticLabel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cv-msg:<SemanticLabel> is deprecated: use cv-msg:SemanticLabel instead.")))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <SemanticLabel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cv-msg:label-val is deprecated.  Use cv-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <SemanticLabel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cv-msg:point-val is deprecated.  Use cv-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SemanticLabel>) ostream)
  "Serializes a message object of type '<SemanticLabel>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SemanticLabel>) istream)
  "Deserializes a message object of type '<SemanticLabel>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SemanticLabel>)))
  "Returns string type for a message object of type '<SemanticLabel>"
  "cv/SemanticLabel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SemanticLabel)))
  "Returns string type for a message object of type 'SemanticLabel"
  "cv/SemanticLabel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SemanticLabel>)))
  "Returns md5sum for a message object of type '<SemanticLabel>"
  "e5c742a84cd4771bb6f0e4d32a54b224")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SemanticLabel)))
  "Returns md5sum for a message object of type 'SemanticLabel"
  "e5c742a84cd4771bb6f0e4d32a54b224")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SemanticLabel>)))
  "Returns full string definition for message of type '<SemanticLabel>"
  (cl:format cl:nil "string label~%geometry_msgs/Point point~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SemanticLabel)))
  "Returns full string definition for message of type 'SemanticLabel"
  (cl:format cl:nil "string label~%geometry_msgs/Point point~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SemanticLabel>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'label))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SemanticLabel>))
  "Converts a ROS message object to a list"
  (cl:list 'SemanticLabel
    (cl:cons ':label (label msg))
    (cl:cons ':point (point msg))
))
