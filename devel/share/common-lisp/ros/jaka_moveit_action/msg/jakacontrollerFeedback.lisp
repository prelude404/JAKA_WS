; Auto-generated. Do not edit!


(cl:in-package jaka_moveit_action-msg)


;//! \htmlinclude jakacontrollerFeedback.msg.html

(cl:defclass <jakacontrollerFeedback> (roslisp-msg-protocol:ros-message)
  ((point_num
    :reader point_num
    :initarg :point_num
    :type cl:fixnum
    :initform 0)
   (robot_now
    :reader robot_now
    :initarg :robot_now
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass jakacontrollerFeedback (<jakacontrollerFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <jakacontrollerFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'jakacontrollerFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jaka_moveit_action-msg:<jakacontrollerFeedback> is deprecated: use jaka_moveit_action-msg:jakacontrollerFeedback instead.")))

(cl:ensure-generic-function 'point_num-val :lambda-list '(m))
(cl:defmethod point_num-val ((m <jakacontrollerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jaka_moveit_action-msg:point_num-val is deprecated.  Use jaka_moveit_action-msg:point_num instead.")
  (point_num m))

(cl:ensure-generic-function 'robot_now-val :lambda-list '(m))
(cl:defmethod robot_now-val ((m <jakacontrollerFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jaka_moveit_action-msg:robot_now-val is deprecated.  Use jaka_moveit_action-msg:robot_now instead.")
  (robot_now m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <jakacontrollerFeedback>) ostream)
  "Serializes a message object of type '<jakacontrollerFeedback>"
  (cl:let* ((signed (cl:slot-value msg 'point_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'robot_now))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'robot_now))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <jakacontrollerFeedback>) istream)
  "Deserializes a message object of type '<jakacontrollerFeedback>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'point_num) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'robot_now) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'robot_now)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<jakacontrollerFeedback>)))
  "Returns string type for a message object of type '<jakacontrollerFeedback>"
  "jaka_moveit_action/jakacontrollerFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'jakacontrollerFeedback)))
  "Returns string type for a message object of type 'jakacontrollerFeedback"
  "jaka_moveit_action/jakacontrollerFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<jakacontrollerFeedback>)))
  "Returns md5sum for a message object of type '<jakacontrollerFeedback>"
  "899863511fa6ada8261fac222bf72461")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'jakacontrollerFeedback)))
  "Returns md5sum for a message object of type 'jakacontrollerFeedback"
  "899863511fa6ada8261fac222bf72461")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<jakacontrollerFeedback>)))
  "Returns full string definition for message of type '<jakacontrollerFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%int16 point_num~%float32[] robot_now~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'jakacontrollerFeedback)))
  "Returns full string definition for message of type 'jakacontrollerFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%int16 point_num~%float32[] robot_now~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <jakacontrollerFeedback>))
  (cl:+ 0
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'robot_now) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <jakacontrollerFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'jakacontrollerFeedback
    (cl:cons ':point_num (point_num msg))
    (cl:cons ':robot_now (robot_now msg))
))
