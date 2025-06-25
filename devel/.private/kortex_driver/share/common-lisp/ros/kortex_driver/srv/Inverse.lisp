; Auto-generated. Do not edit!


(cl:in-package kortex_driver-srv)


;//! \htmlinclude Inverse-request.msg.html

(cl:defclass <Inverse-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Inverse-request (<Inverse-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Inverse-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Inverse-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Inverse-request> is deprecated: use kortex_driver-srv:Inverse-request instead.")))

(cl:ensure-generic-function 'x_position-val :lambda-list '(m))
(cl:defmethod x_position-val ((m <Inverse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:x_position-val is deprecated.  Use kortex_driver-srv:x_position instead.")
  (x_position m))

(cl:ensure-generic-function 'y_position-val :lambda-list '(m))
(cl:defmethod y_position-val ((m <Inverse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:y_position-val is deprecated.  Use kortex_driver-srv:y_position instead.")
  (y_position m))

(cl:ensure-generic-function 'z_position-val :lambda-list '(m))
(cl:defmethod z_position-val ((m <Inverse-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:z_position-val is deprecated.  Use kortex_driver-srv:z_position instead.")
  (z_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Inverse-request>) ostream)
  "Serializes a message object of type '<Inverse-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Inverse-request>) istream)
  "Deserializes a message object of type '<Inverse-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Inverse-request>)))
  "Returns string type for a service object of type '<Inverse-request>"
  "kortex_driver/InverseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Inverse-request)))
  "Returns string type for a service object of type 'Inverse-request"
  "kortex_driver/InverseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Inverse-request>)))
  "Returns md5sum for a message object of type '<Inverse-request>"
  "1652b6a55b9608f7a5bbc015e95332ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Inverse-request)))
  "Returns md5sum for a message object of type 'Inverse-request"
  "1652b6a55b9608f7a5bbc015e95332ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Inverse-request>)))
  "Returns full string definition for message of type '<Inverse-request>"
  (cl:format cl:nil "float64 x_position~%float64 y_position~%float64 z_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Inverse-request)))
  "Returns full string definition for message of type 'Inverse-request"
  (cl:format cl:nil "float64 x_position~%float64 y_position~%float64 z_position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Inverse-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Inverse-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Inverse-request
    (cl:cons ':x_position (x_position msg))
    (cl:cons ':y_position (y_position msg))
    (cl:cons ':z_position (z_position msg))
))
;//! \htmlinclude Inverse-response.msg.html

(cl:defclass <Inverse-response> (roslisp-msg-protocol:ros-message)
  ((joints
    :reader joints
    :initarg :joints
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass Inverse-response (<Inverse-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Inverse-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Inverse-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-srv:<Inverse-response> is deprecated: use kortex_driver-srv:Inverse-response instead.")))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <Inverse-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-srv:joints-val is deprecated.  Use kortex_driver-srv:joints instead.")
  (joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Inverse-response>) ostream)
  "Serializes a message object of type '<Inverse-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joints) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Inverse-response>) istream)
  "Deserializes a message object of type '<Inverse-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joints) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Inverse-response>)))
  "Returns string type for a service object of type '<Inverse-response>"
  "kortex_driver/InverseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Inverse-response)))
  "Returns string type for a service object of type 'Inverse-response"
  "kortex_driver/InverseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Inverse-response>)))
  "Returns md5sum for a message object of type '<Inverse-response>"
  "1652b6a55b9608f7a5bbc015e95332ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Inverse-response)))
  "Returns md5sum for a message object of type 'Inverse-response"
  "1652b6a55b9608f7a5bbc015e95332ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Inverse-response>)))
  "Returns full string definition for message of type '<Inverse-response>"
  (cl:format cl:nil "sensor_msgs/JointState joints~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Inverse-response)))
  "Returns full string definition for message of type 'Inverse-response"
  (cl:format cl:nil "sensor_msgs/JointState joints~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Inverse-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joints))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Inverse-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Inverse-response
    (cl:cons ':joints (joints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Inverse)))
  'Inverse-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Inverse)))
  'Inverse-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Inverse)))
  "Returns string type for a service object of type '<Inverse>"
  "kortex_driver/Inverse")