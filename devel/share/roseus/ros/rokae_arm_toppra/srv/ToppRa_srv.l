;; Auto-generated. Do not edit!


(when (boundp 'rokae_arm_toppra::ToppRa_srv)
  (if (not (find-package "ROKAE_ARM_TOPPRA"))
    (make-package "ROKAE_ARM_TOPPRA"))
  (shadow 'ToppRa_srv (find-package "ROKAE_ARM_TOPPRA")))
(unless (find-package "ROKAE_ARM_TOPPRA::TOPPRA_SRV")
  (make-package "ROKAE_ARM_TOPPRA::TOPPRA_SRV"))
(unless (find-package "ROKAE_ARM_TOPPRA::TOPPRA_SRVREQUEST")
  (make-package "ROKAE_ARM_TOPPRA::TOPPRA_SRVREQUEST"))
(unless (find-package "ROKAE_ARM_TOPPRA::TOPPRA_SRVRESPONSE")
  (make-package "ROKAE_ARM_TOPPRA::TOPPRA_SRVRESPONSE"))

(in-package "ROS")





(defclass rokae_arm_toppra::ToppRa_srvRequest
  :super ros::object
  :slots (_joint_configs_on_way _ifSave ))

(defmethod rokae_arm_toppra::ToppRa_srvRequest
  (:init
   (&key
    ((:joint_configs_on_way __joint_configs_on_way) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ifSave __ifSave) nil)
    )
   (send-super :init)
   (setq _joint_configs_on_way __joint_configs_on_way)
   (setq _ifSave __ifSave)
   self)
  (:joint_configs_on_way
   (&optional __joint_configs_on_way)
   (if __joint_configs_on_way (setq _joint_configs_on_way __joint_configs_on_way)) _joint_configs_on_way)
  (:ifSave
   (&optional __ifSave)
   (if __ifSave (setq _ifSave __ifSave)) _ifSave)
  (:serialization-length
   ()
   (+
    ;; float64[] _joint_configs_on_way
    (* 8    (length _joint_configs_on_way)) 4
    ;; bool _ifSave
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _joint_configs_on_way
     (write-long (length _joint_configs_on_way) s)
     (dotimes (i (length _joint_configs_on_way))
       (sys::poke (elt _joint_configs_on_way i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _ifSave
       (if _ifSave (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _joint_configs_on_way
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_configs_on_way (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_configs_on_way i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _ifSave
     (setq _ifSave (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_arm_toppra::ToppRa_srvResponse
  :super ros::object
  :slots (_pos _vel _acc _t ))

(defmethod rokae_arm_toppra::ToppRa_srvResponse
  (:init
   (&key
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:t __t) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _t __t)
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; float64[] _pos
    (* 8    (length _pos)) 4
    ;; float64[] _vel
    (* 8    (length _vel)) 4
    ;; float64[] _acc
    (* 8    (length _acc)) 4
    ;; float64[] _t
    (* 8    (length _t)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _vel
     (write-long (length _vel) s)
     (dotimes (i (length _vel))
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _acc
     (write-long (length _acc) s)
     (dotimes (i (length _acc))
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _t
     (write-long (length _t) s)
     (dotimes (i (length _t))
       (sys::poke (elt _t i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _acc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acc (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _t
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _t (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _t i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass rokae_arm_toppra::ToppRa_srv
  :super ros::object
  :slots ())

(setf (get rokae_arm_toppra::ToppRa_srv :md5sum-) "7cd293dc242907880f3bae1412ae9b57")
(setf (get rokae_arm_toppra::ToppRa_srv :datatype-) "rokae_arm_toppra/ToppRa_srv")
(setf (get rokae_arm_toppra::ToppRa_srv :request) rokae_arm_toppra::ToppRa_srvRequest)
(setf (get rokae_arm_toppra::ToppRa_srv :response) rokae_arm_toppra::ToppRa_srvResponse)

(defmethod rokae_arm_toppra::ToppRa_srvRequest
  (:response () (instance rokae_arm_toppra::ToppRa_srvResponse :init)))

(setf (get rokae_arm_toppra::ToppRa_srvRequest :md5sum-) "7cd293dc242907880f3bae1412ae9b57")
(setf (get rokae_arm_toppra::ToppRa_srvRequest :datatype-) "rokae_arm_toppra/ToppRa_srvRequest")
(setf (get rokae_arm_toppra::ToppRa_srvRequest :definition-)
      "float64[] joint_configs_on_way 
bool ifSave
---
float64[] pos
float64[] vel
float64[] acc
float64[] t
")

(setf (get rokae_arm_toppra::ToppRa_srvResponse :md5sum-) "7cd293dc242907880f3bae1412ae9b57")
(setf (get rokae_arm_toppra::ToppRa_srvResponse :datatype-) "rokae_arm_toppra/ToppRa_srvResponse")
(setf (get rokae_arm_toppra::ToppRa_srvResponse :definition-)
      "float64[] joint_configs_on_way 
bool ifSave
---
float64[] pos
float64[] vel
float64[] acc
float64[] t
")



(provide :rokae_arm_toppra/ToppRa_srv "7cd293dc242907880f3bae1412ae9b57")


