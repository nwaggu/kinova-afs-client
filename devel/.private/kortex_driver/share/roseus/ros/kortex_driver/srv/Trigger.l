;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Trigger)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Trigger (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::TRIGGER")
  (make-package "KORTEX_DRIVER::TRIGGER"))
(unless (find-package "KORTEX_DRIVER::TRIGGERREQUEST")
  (make-package "KORTEX_DRIVER::TRIGGERREQUEST"))
(unless (find-package "KORTEX_DRIVER::TRIGGERRESPONSE")
  (make-package "KORTEX_DRIVER::TRIGGERRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::TriggerRequest
  :super ros::object
  :slots ())

(defmethod kortex_driver::TriggerRequest
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

(defclass kortex_driver::TriggerResponse
  :super ros::object
  :slots (_success ))

(defmethod kortex_driver::TriggerResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass kortex_driver::Trigger
  :super ros::object
  :slots ())

(setf (get kortex_driver::Trigger :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get kortex_driver::Trigger :datatype-) "kortex_driver/Trigger")
(setf (get kortex_driver::Trigger :request) kortex_driver::TriggerRequest)
(setf (get kortex_driver::Trigger :response) kortex_driver::TriggerResponse)

(defmethod kortex_driver::TriggerRequest
  (:response () (instance kortex_driver::TriggerResponse :init)))

(setf (get kortex_driver::TriggerRequest :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get kortex_driver::TriggerRequest :datatype-) "kortex_driver/TriggerRequest")
(setf (get kortex_driver::TriggerRequest :definition-)
      "
---
bool success

")

(setf (get kortex_driver::TriggerResponse :md5sum-) "358e233cde0c8a8bcfea4ce193f8fc15")
(setf (get kortex_driver::TriggerResponse :datatype-) "kortex_driver/TriggerResponse")
(setf (get kortex_driver::TriggerResponse :definition-)
      "
---
bool success

")



(provide :kortex_driver/Trigger "358e233cde0c8a8bcfea4ce193f8fc15")


