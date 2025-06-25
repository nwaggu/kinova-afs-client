; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude ModeService-request.msg.html

(cl:defclass <ModeService-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform ""))
)

(cl:defclass ModeService-request (<ModeService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ModeService-request> is deprecated: use kortex_driver-srv:ModeService-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ModeService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:mode-val is deprecated.  Use kortex_driver-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeService-request>) ostream)
  "Serializes a message object of type '<ModeService-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeService-request>) istream)
  "Deserializes a message object of type '<ModeService-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeService-request>)))
  "Returns string type for a service object of type '<ModeService-request>"
  "kortex_driver/ModeServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeService-request)))
  "Returns string type for a service object of type 'ModeService-request"
  "kortex_driver/ModeServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeService-request>)))
  "Returns md5sum for a message object of type '<ModeService-request>"
  "939409827c5731ec5346c99677bd2a1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeService-request)))
  "Returns md5sum for a message object of type 'ModeService-request"
  "939409827c5731ec5346c99677bd2a1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeService-request>)))
  "Returns full string definition for message of type '<ModeService-request>"
  (cl:format cl:nil "string mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeService-request)))
  "Returns full string definition for message of type 'ModeService-request"
  (cl:format cl:nil "string mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeService-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeService-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude ModeService-response.msg.html

(cl:defclass <ModeService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ModeService-response (<ModeService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModeService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModeService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<ModeService-response> is deprecated: use kortex_driver-srv:ModeService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ModeService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:success-val is deprecated.  Use kortex_driver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModeService-response>) ostream)
  "Serializes a message object of type '<ModeService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModeService-response>) istream)
  "Deserializes a message object of type '<ModeService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModeService-response>)))
  "Returns string type for a service object of type '<ModeService-response>"
  "kortex_driver/ModeServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeService-response)))
  "Returns string type for a service object of type 'ModeService-response"
  "kortex_driver/ModeServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModeService-response>)))
  "Returns md5sum for a message object of type '<ModeService-response>"
  "939409827c5731ec5346c99677bd2a1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModeService-response)))
  "Returns md5sum for a message object of type 'ModeService-response"
  "939409827c5731ec5346c99677bd2a1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModeService-response>)))
  "Returns full string definition for message of type '<ModeService-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModeService-response)))
  "Returns full string definition for message of type 'ModeService-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModeService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModeService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModeService-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModeService)))
  'ModeService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModeService)))
  'ModeService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModeService)))
  "Returns string type for a service object of type '<ModeService>"
  "kortex_driver/ModeService")