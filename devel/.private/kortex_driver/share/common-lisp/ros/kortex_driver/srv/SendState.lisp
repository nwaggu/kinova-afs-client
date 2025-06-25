; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude SendState-request.msg.html

(cl:defclass <SendState-request> (roslisp-msg-protocol:ros-message)
  ((indicies
    :reader indicies
    :initarg :indicies
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SendState-request (<SendState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendState-request> is deprecated: use kortex_driver-srv:SendState-request instead.")))

(cl:ensure-generic-function 'indicies-val :lambda-list '(m))
(cl:defmethod indicies-val ((m <SendState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:indicies-val is deprecated.  Use kortex_driver-srv:indicies instead.")
  (indicies m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendState-request>) ostream)
  "Serializes a message object of type '<SendState-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'indicies))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'indicies))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendState-request>) istream)
  "Deserializes a message object of type '<SendState-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'indicies) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'indicies)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendState-request>)))
  "Returns string type for a service object of type '<SendState-request>"
  "kortex_driver/SendStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendState-request)))
  "Returns string type for a service object of type 'SendState-request"
  "kortex_driver/SendStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendState-request>)))
  "Returns md5sum for a message object of type '<SendState-request>"
  "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendState-request)))
  "Returns md5sum for a message object of type 'SendState-request"
  "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendState-request>)))
  "Returns full string definition for message of type '<SendState-request>"
  (cl:format cl:nil "int8[] indicies~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendState-request)))
  "Returns full string definition for message of type 'SendState-request"
  (cl:format cl:nil "int8[] indicies~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendState-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'indicies) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendState-request
    (cl:cons ':indicies (indicies msg))
))
;//! \htmlinclude SendState-response.msg.html

(cl:defclass <SendState-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SendState-response (<SendState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<SendState-response> is deprecated: use kortex_driver-srv:SendState-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SendState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:success-val is deprecated.  Use kortex_driver-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendState-response>) ostream)
  "Serializes a message object of type '<SendState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendState-response>) istream)
  "Deserializes a message object of type '<SendState-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendState-response>)))
  "Returns string type for a service object of type '<SendState-response>"
  "kortex_driver/SendStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendState-response)))
  "Returns string type for a service object of type 'SendState-response"
  "kortex_driver/SendStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendState-response>)))
  "Returns md5sum for a message object of type '<SendState-response>"
  "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendState-response)))
  "Returns md5sum for a message object of type 'SendState-response"
  "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendState-response>)))
  "Returns full string definition for message of type '<SendState-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendState-response)))
  "Returns full string definition for message of type 'SendState-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendState-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendState)))
  'SendState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendState)))
  'SendState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendState)))
  "Returns string type for a service object of type '<SendState>"
  "kortex_driver/SendState")