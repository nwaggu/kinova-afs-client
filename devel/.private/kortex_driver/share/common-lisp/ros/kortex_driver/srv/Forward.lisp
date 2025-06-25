; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude Forward-request.msg.html

(cl:defclass <Forward-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Forward-request (<Forward-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Forward-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Forward-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Forward-request> is deprecated: use kortex_driver-srv:Forward-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Forward-request>) ostream)
  "Serializes a message object of type '<Forward-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Forward-request>) istream)
  "Deserializes a message object of type '<Forward-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Forward-request>)))
  "Returns string type for a service object of type '<Forward-request>"
  "kortex_driver/ForwardRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Forward-request)))
  "Returns string type for a service object of type 'Forward-request"
  "kortex_driver/ForwardRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Forward-request>)))
  "Returns md5sum for a message object of type '<Forward-request>"
  "4a65f5a0cdef54b2e1dbb78f7873209a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Forward-request)))
  "Returns md5sum for a message object of type 'Forward-request"
  "4a65f5a0cdef54b2e1dbb78f7873209a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Forward-request>)))
  "Returns full string definition for message of type '<Forward-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Forward-request)))
  "Returns full string definition for message of type 'Forward-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Forward-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Forward-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Forward-request
))
;//! \htmlinclude Forward-response.msg.html

(cl:defclass <Forward-response> (roslisp-msg-protocol:ros-message)
  ((x_position
    :reader x_position
    :initarg :x_position
    :type cl:float
    :initform 0.0)
   (y_position
    :reader y_position
    :initarg :y_position
    :type cl:float
    :initform 0.0)
   (z_position
    :reader z_position
    :initarg :z_position
    :type cl:float
    :initform 0.0))
)

(cl:defclass Forward-response (<Forward-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Forward-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Forward-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Forward-response> is deprecated: use kortex_driver-srv:Forward-response instead.")))

(cl:ensure-generic-function 'x_position-val :lambda-list '(m))
(cl:defmethod x_position-val ((m <Forward-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:x_position-val is deprecated.  Use kortex_driver-srv:x_position instead.")
  (x_position m))

(cl:ensure-generic-function 'y_position-val :lambda-list '(m))
(cl:defmethod y_position-val ((m <Forward-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:y_position-val is deprecated.  Use kortex_driver-srv:y_position instead.")
  (y_position m))

(cl:ensure-generic-function 'z_position-val :lambda-list '(m))
(cl:defmethod z_position-val ((m <Forward-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:z_position-val is deprecated.  Use kortex_driver-srv:z_position instead.")
  (z_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Forward-response>) ostream)
  "Serializes a message object of type '<Forward-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Forward-response>) istream)
  "Deserializes a message object of type '<Forward-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_position) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Forward-response>)))
  "Returns string type for a service object of type '<Forward-response>"
  "kortex_driver/ForwardResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Forward-response)))
  "Returns string type for a service object of type 'Forward-response"
  "kortex_driver/ForwardResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Forward-response>)))
  "Returns md5sum for a message object of type '<Forward-response>"
  "4a65f5a0cdef54b2e1dbb78f7873209a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Forward-response)))
  "Returns md5sum for a message object of type 'Forward-response"
  "4a65f5a0cdef54b2e1dbb78f7873209a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Forward-response>)))
  "Returns full string definition for message of type '<Forward-response>"
  (cl:format cl:nil "float64 x_position~%float64 y_position~%float64 z_position~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Forward-response)))
  "Returns full string definition for message of type 'Forward-response"
  (cl:format cl:nil "float64 x_position~%float64 y_position~%float64 z_position~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Forward-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Forward-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Forward-response
    (cl:cons ':x_position (x_position msg))
    (cl:cons ':y_position (y_position msg))
    (cl:cons ':z_position (z_position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Forward)))
  'Forward-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Forward)))
  'Forward-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Forward)))
  "Returns string type for a service object of type '<Forward>"
  "kortex_driver/Forward")