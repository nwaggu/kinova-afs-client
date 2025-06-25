; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude FindIndex-request.msg.html

(cl:defclass <FindIndex-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass FindIndex-request (<FindIndex-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindIndex-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindIndex-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<FindIndex-request> is deprecated: use kortex_driver-srv:FindIndex-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindIndex-request>) ostream)
  "Serializes a message object of type '<FindIndex-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindIndex-request>) istream)
  "Deserializes a message object of type '<FindIndex-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindIndex-request>)))
  "Returns string type for a service object of type '<FindIndex-request>"
  "kortex_driver/FindIndexRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindIndex-request)))
  "Returns string type for a service object of type 'FindIndex-request"
  "kortex_driver/FindIndexRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindIndex-request>)))
  "Returns md5sum for a message object of type '<FindIndex-request>"
  "dc335e982131613152f901a4d2b0914b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindIndex-request)))
  "Returns md5sum for a message object of type 'FindIndex-request"
  "dc335e982131613152f901a4d2b0914b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindIndex-request>)))
  "Returns full string definition for message of type '<FindIndex-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindIndex-request)))
  "Returns full string definition for message of type 'FindIndex-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindIndex-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindIndex-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FindIndex-request
))
;//! \htmlinclude FindIndex-response.msg.html

(cl:defclass <FindIndex-response> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (grip
    :reader grip
    :initarg :grip
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FindIndex-response (<FindIndex-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FindIndex-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FindIndex-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<FindIndex-response> is deprecated: use kortex_driver-srv:FindIndex-response instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <FindIndex-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:index-val is deprecated.  Use kortex_driver-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'grip-val :lambda-list '(m))
(cl:defmethod grip-val ((m <FindIndex-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:grip-val is deprecated.  Use kortex_driver-srv:grip instead.")
  (grip m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FindIndex-response>) ostream)
  "Serializes a message object of type '<FindIndex-response>"
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'grip)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FindIndex-response>) istream)
  "Deserializes a message object of type '<FindIndex-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'grip) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FindIndex-response>)))
  "Returns string type for a service object of type '<FindIndex-response>"
  "kortex_driver/FindIndexResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindIndex-response)))
  "Returns string type for a service object of type 'FindIndex-response"
  "kortex_driver/FindIndexResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FindIndex-response>)))
  "Returns md5sum for a message object of type '<FindIndex-response>"
  "dc335e982131613152f901a4d2b0914b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FindIndex-response)))
  "Returns md5sum for a message object of type 'FindIndex-response"
  "dc335e982131613152f901a4d2b0914b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FindIndex-response>)))
  "Returns full string definition for message of type '<FindIndex-response>"
  (cl:format cl:nil "int8 index~%int8 grip~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FindIndex-response)))
  "Returns full string definition for message of type 'FindIndex-response"
  (cl:format cl:nil "int8 index~%int8 grip~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FindIndex-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FindIndex-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FindIndex-response
    (cl:cons ':index (index msg))
    (cl:cons ':grip (grip msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FindIndex)))
  'FindIndex-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FindIndex)))
  'FindIndex-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FindIndex)))
  "Returns string type for a service object of type '<FindIndex>"
  "kortex_driver/FindIndex")