;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ModeService)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ModeService (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::MODESERVICE")
  (make-package "KORTEX_DRIVER::MODESERVICE"))
(unless (find-package "KORTEX_DRIVER::MODESERVICEREQUEST")
  (make-package "KORTEX_DRIVER::MODESERVICEREQUEST"))
(unless (find-package "KORTEX_DRIVER::MODESERVICERESPONSE")
  (make-package "KORTEX_DRIVER::MODESERVICERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::ModeServiceRequest
  :super ros::object
  :slots (_mode ))

(defmethod kortex_driver::ModeServiceRequest
  (:init
   (&key
    ((:mode __mode) "")
    )
   (send-super :init)
   (setq _mode (string __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass kortex_driver::ModeServiceResponse
  :super ros::object
  :slots (_success ))

(defmethod kortex_driver::ModeServiceResponse
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

(defclass kortex_driver::ModeService
  :super ros::object
  :slots ())

(setf (get kortex_driver::ModeService :md5sum-) "939409827c5731ec5346c99677bd2a1d")
(setf (get kortex_driver::ModeService :datatype-) "kortex_driver/ModeService")
(setf (get kortex_driver::ModeService :request) kortex_driver::ModeServiceRequest)
(setf (get kortex_driver::ModeService :response) kortex_driver::ModeServiceResponse)

(defmethod kortex_driver::ModeServiceRequest
  (:response () (instance kortex_driver::ModeServiceResponse :init)))

(setf (get kortex_driver::ModeServiceRequest :md5sum-) "939409827c5731ec5346c99677bd2a1d")
(setf (get kortex_driver::ModeServiceRequest :datatype-) "kortex_driver/ModeServiceRequest")
(setf (get kortex_driver::ModeServiceRequest :definition-)
      "string mode
---
bool success

")

(setf (get kortex_driver::ModeServiceResponse :md5sum-) "939409827c5731ec5346c99677bd2a1d")
(setf (get kortex_driver::ModeServiceResponse :datatype-) "kortex_driver/ModeServiceResponse")
(setf (get kortex_driver::ModeServiceResponse :definition-)
      "string mode
---
bool success

")



(provide :kortex_driver/ModeService "939409827c5731ec5346c99677bd2a1d")


