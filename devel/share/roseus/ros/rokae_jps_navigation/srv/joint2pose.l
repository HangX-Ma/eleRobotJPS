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



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rokae_jps_navigation::joint2poseRequest
  :super ros::object
  :slots (_joint0 _joint1 _joint2 _joint3 _joint4 _joint5 _ifVerbose ))

(defmethod rokae_jps_navigation::joint2poseRequest
  (:init
   (&key
    ((:joint0 __joint0) 0.0)
    ((:joint1 __joint1) 0.0)
    ((:joint2 __joint2) 0.0)
    ((:joint3 __joint3) 0.0)
    ((:joint4 __joint4) 0.0)
    ((:joint5 __joint5) 0.0)
    ((:ifVerbose __ifVerbose) nil)
    )
   (send-super :init)
   (setq _joint0 (float __joint0))
   (setq _joint1 (float __joint1))
   (setq _joint2 (float __joint2))
   (setq _joint3 (float __joint3))
   (setq _joint4 (float __joint4))
   (setq _joint5 (float __joint5))
   (setq _ifVerbose __ifVerbose)
   self)
  (:joint0
   (&optional __joint0)
   (if __joint0 (setq _joint0 __joint0)) _joint0)
  (:joint1
   (&optional __joint1)
   (if __joint1 (setq _joint1 __joint1)) _joint1)
  (:joint2
   (&optional __joint2)
   (if __joint2 (setq _joint2 __joint2)) _joint2)
  (:joint3
   (&optional __joint3)
   (if __joint3 (setq _joint3 __joint3)) _joint3)
  (:joint4
   (&optional __joint4)
   (if __joint4 (setq _joint4 __joint4)) _joint4)
  (:joint5
   (&optional __joint5)
   (if __joint5 (setq _joint5 __joint5)) _joint5)
  (:ifVerbose
   (&optional __ifVerbose)
   (if __ifVerbose (setq _ifVerbose __ifVerbose)) _ifVerbose)
  (:serialization-length
   ()
   (+
    ;; float64 _joint0
    8
    ;; float64 _joint1
    8
    ;; float64 _joint2
    8
    ;; float64 _joint3
    8
    ;; float64 _joint4
    8
    ;; float64 _joint5
    8
    ;; bool _ifVerbose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _joint0
       (sys::poke _joint0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint1
       (sys::poke _joint1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint2
       (sys::poke _joint2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint3
       (sys::poke _joint3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint4
       (sys::poke _joint4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _joint5
       (sys::poke _joint5 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _ifVerbose
       (if _ifVerbose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _joint0
     (setq _joint0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint1
     (setq _joint1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint2
     (setq _joint2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint3
     (setq _joint3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint4
     (setq _joint4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _joint5
     (setq _joint5 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _ifVerbose
     (setq _ifVerbose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_jps_navigation::joint2poseResponse
  :super ros::object
  :slots (_re_pose ))

(defmethod rokae_jps_navigation::joint2poseResponse
  (:init
   (&key
    ((:re_pose __re_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _re_pose __re_pose)
   self)
  (:re_pose
   (&rest __re_pose)
   (if (keywordp (car __re_pose))
       (send* _re_pose __re_pose)
     (progn
       (if __re_pose (setq _re_pose (car __re_pose)))
       _re_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _re_pose
    (send _re_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _re_pose
       (send _re_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _re_pose
     (send _re_pose :deserialize buf ptr-) (incf ptr- (send _re_pose :serialization-length))
   ;;
   self)
  )

(defclass rokae_jps_navigation::joint2pose
  :super ros::object
  :slots ())

(setf (get rokae_jps_navigation::joint2pose :md5sum-) "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(setf (get rokae_jps_navigation::joint2pose :datatype-) "rokae_jps_navigation/joint2pose")
(setf (get rokae_jps_navigation::joint2pose :request) rokae_jps_navigation::joint2poseRequest)
(setf (get rokae_jps_navigation::joint2pose :response) rokae_jps_navigation::joint2poseResponse)

(defmethod rokae_jps_navigation::joint2poseRequest
  (:response () (instance rokae_jps_navigation::joint2poseResponse :init)))

(setf (get rokae_jps_navigation::joint2poseRequest :md5sum-) "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(setf (get rokae_jps_navigation::joint2poseRequest :datatype-) "rokae_jps_navigation/joint2poseRequest")
(setf (get rokae_jps_navigation::joint2poseRequest :definition-)
      "float64 joint0
float64 joint1
float64 joint2
float64 joint3
float64 joint4
float64 joint5
bool ifVerbose
---
geometry_msgs/Pose re_pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get rokae_jps_navigation::joint2poseResponse :md5sum-) "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(setf (get rokae_jps_navigation::joint2poseResponse :datatype-) "rokae_jps_navigation/joint2poseResponse")
(setf (get rokae_jps_navigation::joint2poseResponse :definition-)
      "float64 joint0
float64 joint1
float64 joint2
float64 joint3
float64 joint4
float64 joint5
bool ifVerbose
---
geometry_msgs/Pose re_pose


================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :rokae_jps_navigation/joint2pose "8e3735eb3bfe1ca9ba91861cd4e5d3f8")


