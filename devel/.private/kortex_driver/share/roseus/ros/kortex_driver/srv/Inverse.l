;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Inverse)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Inverse (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INVERSE")
  (make-package "KORTEX_DRIVER::INVERSE"))
(unless (find-package "KORTEX_DRIVER::INVERSEREQUEST")
  (make-package "KORTEX_DRIVER::INVERSEREQUEST"))
(unless (find-package "KORTEX_DRIVER::INVERSERESPONSE")
  (make-package "KORTEX_DRIVER::INVERSERESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass kortex_driver::InverseRequest
  :super ros::object
  :slots (_x_position _y_position _z_position ))

(defmethod kortex_driver::InverseRequest
  (:init
   (&key
    ((:x_position __x_position) 0.0)
    ((:y_position __y_position) 0.0)
    ((:z_position __z_position) 0.0)
    )
   (send-super :init)
   (setq _x_position (float __x_position))
   (setq _y_position (float __y_position))
   (setq _z_position (float __z_position))
   self)
  (:x_position
   (&optional __x_position)
   (if __x_position (setq _x_position __x_position)) _x_position)
  (:y_position
   (&optional __y_position)
   (if __y_position (setq _y_position __y_position)) _y_position)
  (:z_position
   (&optional __z_position)
   (if __z_position (setq _z_position __z_position)) _z_position)
  (:serialization-length
   ()
   (+
    ;; float64 _x_position
    8
    ;; float64 _y_position
    8
    ;; float64 _z_position
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_position
       (sys::poke _x_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_position
       (sys::poke _y_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_position
       (sys::poke _z_position (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_position
     (setq _x_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_position
     (setq _y_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_position
     (setq _z_position (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass kortex_driver::InverseResponse
  :super ros::object
  :slots (_joints ))

(defmethod kortex_driver::InverseResponse
  (:init
   (&key
    ((:joints __joints) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _joints __joints)
   self)
  (:joints
   (&rest __joints)
   (if (keywordp (car __joints))
       (send* _joints __joints)
     (progn
       (if __joints (setq _joints (car __joints)))
       _joints)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/JointState _joints
    (send _joints :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/JointState _joints
       (send _joints :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/JointState _joints
     (send _joints :deserialize buf ptr-) (incf ptr- (send _joints :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::Inverse
  :super ros::object
  :slots ())

(setf (get kortex_driver::Inverse :md5sum-) "1652b6a55b9608f7a5bbc015e95332ff")
(setf (get kortex_driver::Inverse :datatype-) "kortex_driver/Inverse")
(setf (get kortex_driver::Inverse :request) kortex_driver::InverseRequest)
(setf (get kortex_driver::Inverse :response) kortex_driver::InverseResponse)

(defmethod kortex_driver::InverseRequest
  (:response () (instance kortex_driver::InverseResponse :init)))

(setf (get kortex_driver::InverseRequest :md5sum-) "1652b6a55b9608f7a5bbc015e95332ff")
(setf (get kortex_driver::InverseRequest :datatype-) "kortex_driver/InverseRequest")
(setf (get kortex_driver::InverseRequest :definition-)
      "float64 x_position
float64 y_position
float64 z_position
---
sensor_msgs/JointState joints


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
")

(setf (get kortex_driver::InverseResponse :md5sum-) "1652b6a55b9608f7a5bbc015e95332ff")
(setf (get kortex_driver::InverseResponse :datatype-) "kortex_driver/InverseResponse")
(setf (get kortex_driver::InverseResponse :definition-)
      "float64 x_position
float64 y_position
float64 z_position
---
sensor_msgs/JointState joints


================================================================================
MSG: sensor_msgs/JointState
# This is a message that holds data to describe the state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
")



(provide :kortex_driver/Inverse "1652b6a55b9608f7a5bbc015e95332ff")


