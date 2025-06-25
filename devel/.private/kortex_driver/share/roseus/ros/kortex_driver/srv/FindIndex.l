;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FindIndex)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FindIndex (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FINDINDEX")
  (make-package "KORTEX_DRIVER::FINDINDEX"))
(unless (find-package "KORTEX_DRIVER::FINDINDEXREQUEST")
  (make-package "KORTEX_DRIVER::FINDINDEXREQUEST"))
(unless (find-package "KORTEX_DRIVER::FINDINDEXRESPONSE")
  (make-package "KORTEX_DRIVER::FINDINDEXRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::FindIndexRequest
  :super ros::object
  :slots ())

(defmethod kortex_driver::FindIndexRequest
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

(defclass kortex_driver::FindIndexResponse
  :super ros::object
  :slots (_index _grip ))

(defmethod kortex_driver::FindIndexResponse
  (:init
   (&key
    ((:index __index) 0)
    ((:grip __grip) 0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _grip (round __grip))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:grip
   (&optional __grip)
   (if __grip (setq _grip __grip)) _grip)
  (:serialization-length
   ()
   (+
    ;; int8 _index
    1
    ;; int8 _grip
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _index
       (write-byte _index s)
     ;; int8 _grip
       (write-byte _grip s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _index
     (setq _index (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _index 127) (setq _index (- _index 256)))
   ;; int8 _grip
     (setq _grip (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _grip 127) (setq _grip (- _grip 256)))
   ;;
   self)
  )

(defclass kortex_driver::FindIndex
  :super ros::object
  :slots ())

(setf (get kortex_driver::FindIndex :md5sum-) "dc335e982131613152f901a4d2b0914b")
(setf (get kortex_driver::FindIndex :datatype-) "kortex_driver/FindIndex")
(setf (get kortex_driver::FindIndex :request) kortex_driver::FindIndexRequest)
(setf (get kortex_driver::FindIndex :response) kortex_driver::FindIndexResponse)

(defmethod kortex_driver::FindIndexRequest
  (:response () (instance kortex_driver::FindIndexResponse :init)))

(setf (get kortex_driver::FindIndexRequest :md5sum-) "dc335e982131613152f901a4d2b0914b")
(setf (get kortex_driver::FindIndexRequest :datatype-) "kortex_driver/FindIndexRequest")
(setf (get kortex_driver::FindIndexRequest :definition-)
      "
---
int8 index
int8 grip

")

(setf (get kortex_driver::FindIndexResponse :md5sum-) "dc335e982131613152f901a4d2b0914b")
(setf (get kortex_driver::FindIndexResponse :datatype-) "kortex_driver/FindIndexResponse")
(setf (get kortex_driver::FindIndexResponse :definition-)
      "
---
int8 index
int8 grip

")



(provide :kortex_driver/FindIndex "dc335e982131613152f901a4d2b0914b")


