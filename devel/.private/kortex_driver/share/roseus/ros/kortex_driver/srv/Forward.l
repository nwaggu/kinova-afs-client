;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Forward)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Forward (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FORWARD")
  (make-package "KORTEX_DRIVER::FORWARD"))
(unless (find-package "KORTEX_DRIVER::FORWARDREQUEST")
  (make-package "KORTEX_DRIVER::FORWARDREQUEST"))
(unless (find-package "KORTEX_DRIVER::FORWARDRESPONSE")
  (make-package "KORTEX_DRIVER::FORWARDRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ForwardRequest
  :super ros::object
  :slots ())

(defmethod kortex_driver::ForwardRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass kortex_driver::ForwardResponse
  :super ros::object
  :slots (_x_position _y_position _z_position ))

(defmethod kortex_driver::ForwardResponse
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

(defclass kortex_driver::Forward
  :super ros::object
  :slots ())

(setf (get kortex_driver::Forward :md5sum-) "4a65f5a0cdef54b2e1dbb78f7873209a")
(setf (get kortex_driver::Forward :datatype-) "kortex_driver/Forward")
(setf (get kortex_driver::Forward :request) kortex_driver::ForwardRequest)
(setf (get kortex_driver::Forward :response) kortex_driver::ForwardResponse)

(defmethod kortex_driver::ForwardRequest
  (:response () (instance kortex_driver::ForwardResponse :init)))

(setf (get kortex_driver::ForwardRequest :md5sum-) "4a65f5a0cdef54b2e1dbb78f7873209a")
(setf (get kortex_driver::ForwardRequest :datatype-) "kortex_driver/ForwardRequest")
(setf (get kortex_driver::ForwardRequest :definition-)
      "
---
float64 x_position
float64 y_position
float64 z_position

")

(setf (get kortex_driver::ForwardResponse :md5sum-) "4a65f5a0cdef54b2e1dbb78f7873209a")
(setf (get kortex_driver::ForwardResponse :datatype-) "kortex_driver/ForwardResponse")
(setf (get kortex_driver::ForwardResponse :definition-)
      "
---
float64 x_position
float64 y_position
float64 z_position

")



(provide :kortex_driver/Forward "4a65f5a0cdef54b2e1dbb78f7873209a")


