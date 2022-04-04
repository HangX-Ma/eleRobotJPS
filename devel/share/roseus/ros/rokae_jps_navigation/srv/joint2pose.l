;; Auto-generated. Do not edit!


(when (boundp 'rokae_jps_navigation::joint2pose)
  (if (not (find-package "ROKAE_JPS_NAVIGATION"))
    (make-package "ROKAE_JPS_NAVIGATION"))
  (shadow 'joint2pose (find-package "ROKAE_JPS_NAVIGATION")))
(unless (find-package "ROKAE_JPS_NAVIGATION::JOINT2POSE")
  (make-package "ROKAE_JPS_NAVIGATION::JOINT2POSE"))
(unless (find-package "ROKAE_JPS_NAVIGATION::JOINT2POSEREQUEST")
  (make-package "ROKAE_JPS_NAVIGATION::JOINT2POSEREQUEST"))
(unless (find-package "ROKAE_JPS_NAVIGATION::JOINT2POSERESPONSE")
  (make-package "ROKAE_JPS_NAVIGATION::JOINT2POSERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass rokae_jps_navigation::joint2poseRequest
  :super ros::object
  :slots (_joint0 _joint1 _joint2 _joint3 _joint4 _joint5 _ifVerbose ))

(defmethod rokae_jps_navigation::joint2poseRequest
  (:init
   (&key
    ((:joint0 __joint0) (instance std_msgs::Float32 :init))
    ((:joint1 __joint1) (instance std_msgs::Float32 :init))
    ((:joint2 __joint2) (instance std_msgs::Float32 :init))
    ((:joint3 __joint3) (instance std_msgs::Float32 :init))
    ((:joint4 __joint4) (instance std_msgs::Float32 :init))
    ((:joint5 __joint5) (instance std_msgs::Float32 :init))
    ((:ifVerbose __ifVerbose) nil)
    )
   (send-super :init)
   (setq _joint0 __joint0)
   (setq _joint1 __joint1)
   (setq _joint2 __joint2)
   (setq _joint3 __joint3)
   (setq _joint4 __joint4)
   (setq _joint5 __joint5)
   (setq _ifVerbose __ifVerbose)
   self)
  (:joint0
   (&rest __joint0)
   (if (keywordp (car __joint0))
       (send* _joint0 __joint0)
     (progn
       (if __joint0 (setq _joint0 (car __joint0)))
       _joint0)))
  (:joint1
   (&rest __joint1)
   (if (keywordp (car __joint1))
       (send* _joint1 __joint1)
     (progn
       (if __joint1 (setq _joint1 (car __joint1)))
       _joint1)))
  (:joint2
   (&rest __joint2)
   (if (keywordp (car __joint2))
       (send* _joint2 __joint2)
     (progn
       (if __joint2 (setq _joint2 (car __joint2)))
       _joint2)))
  (:joint3
   (&rest __joint3)
   (if (keywordp (car __joint3))
       (send* _joint3 __joint3)
     (progn
       (if __joint3 (setq _joint3 (car __joint3)))
       _joint3)))
  (:joint4
   (&rest __joint4)
   (if (keywordp (car __joint4))
       (send* _joint4 __joint4)
     (progn
       (if __joint4 (setq _joint4 (car __joint4)))
       _joint4)))
  (:joint5
   (&rest __joint5)
   (if (keywordp (car __joint5))
       (send* _joint5 __joint5)
     (progn
       (if __joint5 (setq _joint5 (car __joint5)))
       _joint5)))
  (:ifVerbose
   (&optional __ifVerbose)
   (if __ifVerbose (setq _ifVerbose __ifVerbose)) _ifVerbose)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32 _joint0
    (send _joint0 :serialization-length)
    ;; std_msgs/Float32 _joint1
    (send _joint1 :serialization-length)
    ;; std_msgs/Float32 _joint2
    (send _joint2 :serialization-length)
    ;; std_msgs/Float32 _joint3
    (send _joint3 :serialization-length)
    ;; std_msgs/Float32 _joint4
    (send _joint4 :serialization-length)
    ;; std_msgs/Float32 _joint5
    (send _joint5 :serialization-length)
    ;; bool _ifVerbose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32 _joint0
       (send _joint0 :serialize s)
     ;; std_msgs/Float32 _joint1
       (send _joint1 :serialize s)
     ;; std_msgs/Float32 _joint2
       (send _joint2 :serialize s)
     ;; std_msgs/Float32 _joint3
       (send _joint3 :serialize s)
     ;; std_msgs/Float32 _joint4
       (send _joint4 :serialize s)
     ;; std_msgs/Float32 _joint5
       (send _joint5 :serialize s)
     ;; bool _ifVerbose
       (if _ifVerbose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32 _joint0
     (send _joint0 :deserialize buf ptr-) (incf ptr- (send _joint0 :serialization-length))
   ;; std_msgs/Float32 _joint1
     (send _joint1 :deserialize buf ptr-) (incf ptr- (send _joint1 :serialization-length))
   ;; std_msgs/Float32 _joint2
     (send _joint2 :deserialize buf ptr-) (incf ptr- (send _joint2 :serialization-length))
   ;; std_msgs/Float32 _joint3
     (send _joint3 :deserialize buf ptr-) (incf ptr- (send _joint3 :serialization-length))
   ;; std_msgs/Float32 _joint4
     (send _joint4 :deserialize buf ptr-) (incf ptr- (send _joint4 :serialization-length))
   ;; std_msgs/Float32 _joint5
     (send _joint5 :deserialize buf ptr-) (incf ptr- (send _joint5 :serialization-length))
   ;; bool _ifVerbose
     (setq _ifVerbose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_jps_navigation::joint2poseResponse
  :super ros::object
  :slots ())

(defmethod rokae_jps_navigation::joint2poseResponse
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

(defclass rokae_jps_navigation::joint2pose
  :super ros::object
  :slots ())

(setf (get rokae_jps_navigation::joint2pose :md5sum-) "5999e34ffaace7b38734bfad5de479e9")
(setf (get rokae_jps_navigation::joint2pose :datatype-) "rokae_jps_navigation/joint2pose")
(setf (get rokae_jps_navigation::joint2pose :request) rokae_jps_navigation::joint2poseRequest)
(setf (get rokae_jps_navigation::joint2pose :response) rokae_jps_navigation::joint2poseResponse)

(defmethod rokae_jps_navigation::joint2poseRequest
  (:response () (instance rokae_jps_navigation::joint2poseResponse :init)))

(setf (get rokae_jps_navigation::joint2poseRequest :md5sum-) "5999e34ffaace7b38734bfad5de479e9")
(setf (get rokae_jps_navigation::joint2poseRequest :datatype-) "rokae_jps_navigation/joint2poseRequest")
(setf (get rokae_jps_navigation::joint2poseRequest :definition-)
      "std_msgs/Float32 joint0
std_msgs/Float32 joint1
std_msgs/Float32 joint2
std_msgs/Float32 joint3
std_msgs/Float32 joint4
std_msgs/Float32 joint5
bool ifVerbose

================================================================================
MSG: std_msgs/Float32
---

")

(setf (get rokae_jps_navigation::joint2poseResponse :md5sum-) "5999e34ffaace7b38734bfad5de479e9")
(setf (get rokae_jps_navigation::joint2poseResponse :datatype-) "rokae_jps_navigation/joint2poseResponse")
(setf (get rokae_jps_navigation::joint2poseResponse :definition-)
      "std_msgs/Float32 joint0
std_msgs/Float32 joint1
std_msgs/Float32 joint2
std_msgs/Float32 joint3
std_msgs/Float32 joint4
std_msgs/Float32 joint5
bool ifVerbose

================================================================================
MSG: std_msgs/Float32
---

")



(provide :rokae_jps_navigation/joint2pose "5999e34ffaace7b38734bfad5de479e9")


