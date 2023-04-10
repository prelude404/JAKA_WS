; Auto-generated. Do not edit!


(cl:in-package ur_planning-srv)


;//! \htmlinclude grasp_pose-request.msg.html

(cl:defclass <grasp_pose-request> (roslisp-msg-protocol:ros-message)
  ((grasppose_x
    :reader grasppose_x
    :initarg :grasppose_x
    :type cl:float
    :initform 0.0)
   (grasppose_y
    :reader grasppose_y
    :initarg :grasppose_y
    :type cl:float
    :initform 0.0)
   (grasppose_z
    :reader grasppose_z
    :initarg :grasppose_z
    :type cl:float
    :initform 0.0)
   (grasppose_R
    :reader grasppose_R
    :initarg :grasppose_R
    :type cl:float
    :initform 0.0)
   (grasppose_P
    :reader grasppose_P
    :initarg :grasppose_P
    :type cl:float
    :initform 0.0)
   (grasppose_Y
    :reader grasppose_Y
    :initarg :grasppose_Y
    :type cl:float
    :initform 0.0))
)

(cl:defclass grasp_pose-request (<grasp_pose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grasp_pose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grasp_pose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_planning-srv:<grasp_pose-request> is deprecated: use ur_planning-srv:grasp_pose-request instead.")))

(cl:ensure-generic-function 'grasppose_x-val :lambda-list '(m))
(cl:defmethod grasppose_x-val ((m <grasp_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:grasppose_x-val is deprecated.  Use ur_planning-srv:grasppose_x instead.")
  (grasppose_x m))

(cl:ensure-generic-function 'grasppose_y-val :lambda-list '(m))
(cl:defmethod grasppose_y-val ((m <grasp_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:grasppose_y-val is deprecated.  Use ur_planning-srv:grasppose_y instead.")
  (grasppose_y m))

(cl:ensure-generic-function 'grasppose_z-val :lambda-list '(m))
(cl:defmethod grasppose_z-val ((m <grasp_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:grasppose_z-val is deprecated.  Use ur_planning-srv:grasppose_z instead.")
  (grasppose_z m))

(cl:ensure-generic-function 'grasppose_R-val :lambda-list '(m))
(cl:defmethod grasppose_R-val ((m <grasp_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:grasppose_R-val is deprecated.  Use ur_planning-srv:grasppose_R instead.")
  (grasppose_R m))

(cl:ensure-generic-function 'grasppose_P-val :lambda-list '(m))
(cl:defmethod grasppose_P-val ((m <grasp_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:grasppose_P-val is deprecated.  Use ur_planning-srv:grasppose_P instead.")
  (grasppose_P m))

(cl:ensure-generic-function 'grasppose_Y-val :lambda-list '(m))
(cl:defmethod grasppose_Y-val ((m <grasp_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:grasppose_Y-val is deprecated.  Use ur_planning-srv:grasppose_Y instead.")
  (grasppose_Y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grasp_pose-request>) ostream)
  "Serializes a message object of type '<grasp_pose-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grasppose_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grasppose_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grasppose_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grasppose_R))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grasppose_P))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grasppose_Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grasp_pose-request>) istream)
  "Deserializes a message object of type '<grasp_pose-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasppose_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasppose_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasppose_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasppose_R) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasppose_P) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasppose_Y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grasp_pose-request>)))
  "Returns string type for a service object of type '<grasp_pose-request>"
  "ur_planning/grasp_poseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp_pose-request)))
  "Returns string type for a service object of type 'grasp_pose-request"
  "ur_planning/grasp_poseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grasp_pose-request>)))
  "Returns md5sum for a message object of type '<grasp_pose-request>"
  "8a5978aa25b0b059ee08ab4584d6c721")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grasp_pose-request)))
  "Returns md5sum for a message object of type 'grasp_pose-request"
  "8a5978aa25b0b059ee08ab4584d6c721")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grasp_pose-request>)))
  "Returns full string definition for message of type '<grasp_pose-request>"
  (cl:format cl:nil "float64 grasppose_x~%float64 grasppose_y~%float64 grasppose_z~%float64 grasppose_R~%float64 grasppose_P~%float64 grasppose_Y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grasp_pose-request)))
  "Returns full string definition for message of type 'grasp_pose-request"
  (cl:format cl:nil "float64 grasppose_x~%float64 grasppose_y~%float64 grasppose_z~%float64 grasppose_R~%float64 grasppose_P~%float64 grasppose_Y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grasp_pose-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grasp_pose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'grasp_pose-request
    (cl:cons ':grasppose_x (grasppose_x msg))
    (cl:cons ':grasppose_y (grasppose_y msg))
    (cl:cons ':grasppose_z (grasppose_z msg))
    (cl:cons ':grasppose_R (grasppose_R msg))
    (cl:cons ':grasppose_P (grasppose_P msg))
    (cl:cons ':grasppose_Y (grasppose_Y msg))
))
;//! \htmlinclude grasp_pose-response.msg.html

(cl:defclass <grasp_pose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass grasp_pose-response (<grasp_pose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grasp_pose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grasp_pose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_planning-srv:<grasp_pose-response> is deprecated: use ur_planning-srv:grasp_pose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <grasp_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_planning-srv:success-val is deprecated.  Use ur_planning-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grasp_pose-response>) ostream)
  "Serializes a message object of type '<grasp_pose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grasp_pose-response>) istream)
  "Deserializes a message object of type '<grasp_pose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grasp_pose-response>)))
  "Returns string type for a service object of type '<grasp_pose-response>"
  "ur_planning/grasp_poseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp_pose-response)))
  "Returns string type for a service object of type 'grasp_pose-response"
  "ur_planning/grasp_poseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grasp_pose-response>)))
  "Returns md5sum for a message object of type '<grasp_pose-response>"
  "8a5978aa25b0b059ee08ab4584d6c721")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grasp_pose-response)))
  "Returns md5sum for a message object of type 'grasp_pose-response"
  "8a5978aa25b0b059ee08ab4584d6c721")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grasp_pose-response>)))
  "Returns full string definition for message of type '<grasp_pose-response>"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grasp_pose-response)))
  "Returns full string definition for message of type 'grasp_pose-response"
  (cl:format cl:nil "bool success~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grasp_pose-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grasp_pose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'grasp_pose-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'grasp_pose)))
  'grasp_pose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'grasp_pose)))
  'grasp_pose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp_pose)))
  "Returns string type for a service object of type '<grasp_pose>"
  "ur_planning/grasp_pose")