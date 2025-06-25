;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SendState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SendState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SENDSTATE")
  (make-package "KORTEX_DRIVER::SENDSTATE"))
(unless (find-package "KORTEX_DRIVER::SENDSTATEREQUEST")
  (make-package "KORTEX_DRIVER::SENDSTATEREQUEST"))
(unless (find-package "KORTEX_DRIVER::SENDSTATERESPONSE")
  (make-package "KORTEX_DRIVER::SENDSTATERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::SendStateRequest
  :super ros::object
  :slots (_indicies ))

(defmethod kortex_driver::SendStateRequest
  (:init
   (&key
    ((:indicies __indicies) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _indicies __indicies)
   self)
  (:indicies
   (&optional __indicies)
   (if __indicies (setq _indicies __indicies)) _indicies)
  (:serialization-length
   ()
   (+
    ;; int8[] _indicies
    (* 1    (length _indicies)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8[] _indicies
     (write-long (length _indicies) s)
     (dotimes (i (length _indicies))
       (write-byte (elt _indicies i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8[] _indicies
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _indicies (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _indicies i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _indicies i) 127) (setf (elt _indicies i) (- (elt _indicies i) 256)))
     ))
   ;;
   self)
  )

(defclass kortex_driver::SendStateResponse
  :super ros::object
  :slots (_success ))

(defmethod kortex_driver::SendStateResponse
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

(defclass kortex_driver::SendState
  :super ros::object
  :slots ())

(setf (get kortex_driver::SendState :md5sum-) "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(setf (get kortex_driver::SendState :datatype-) "kortex_driver/SendState")
(setf (get kortex_driver::SendState :request) kortex_driver::SendStateRequest)
(setf (get kortex_driver::SendState :response) kortex_driver::SendStateResponse)

(defmethod kortex_driver::SendStateRequest
  (:response () (instance kortex_driver::SendStateResponse :init)))

(setf (get kortex_driver::SendStateRequest :md5sum-) "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(setf (get kortex_driver::SendStateRequest :datatype-) "kortex_driver/SendStateRequest")
(setf (get kortex_driver::SendStateRequest :definition-)
      "int8[] indicies
---
bool success

")

(setf (get kortex_driver::SendStateResponse :md5sum-) "2db8f4ad9fe7fd5f3afc52b1481c1ea1")
(setf (get kortex_driver::SendStateResponse :datatype-) "kortex_driver/SendStateResponse")
(setf (get kortex_driver::SendStateResponse :definition-)
      "int8[] indicies
---
bool success

")



(provide :kortex_driver/SendState "2db8f4ad9fe7fd5f3afc52b1481c1ea1")


