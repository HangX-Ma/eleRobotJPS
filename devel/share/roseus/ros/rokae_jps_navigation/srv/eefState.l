;; Auto-generated. Do not edit!


(when (boundp 'rokae_jps_navigation::eefState)
  (if (not (find-package "ROKAE_JPS_NAVIGATION"))
    (make-package "ROKAE_JPS_NAVIGATION"))
  (shadow 'eefState (find-package "ROKAE_JPS_NAVIGATION")))
(unless (find-package "ROKAE_JPS_NAVIGATION::EEFSTATE")
  (make-package "ROKAE_JPS_NAVIGATION::EEFSTATE"))
(unless (find-package "ROKAE_JPS_NAVIGATION::EEFSTATEREQUEST")
  (make-package "ROKAE_JPS_NAVIGATION::EEFSTATEREQUEST"))
(unless (find-package "ROKAE_JPS_NAVIGATION::EEFSTATERESPONSE")
  (make-package "ROKAE_JPS_NAVIGATION::EEFSTATERESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass rokae_jps_navigation::eefStateRequest
  :super ros::object
  :slots (_ifVerbose ))

(defmethod rokae_jps_navigation::eefStateRequest
  (:init
   (&key
    ((:ifVerbose __ifVerbose) nil)
    )
   (send-super :init)
   (setq _ifVerbose __ifVerbose)
   self)
  (:ifVerbose
   (&optional __ifVerbose)
   (if __ifVerbose (setq _ifVerbose __ifVerbose)) _ifVerbose)
  (:serialization-length
   ()
   (+
    ;; bool _ifVerbose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ifVerbose
       (if _ifVerbose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ifVerbose
     (setq _ifVerbose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_jps_navigation::eefStateResponse
  :super ros::object
  :slots (_eef_pose ))

(defmethod rokae_jps_navigation::eefStateResponse
  (:init
   (&key
    ((:eef_pose __eef_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _eef_pose __eef_pose)
   self)
  (:eef_pose
   (&rest __eef_pose)
   (if (keywordp (car __eef_pose))
       (send* _eef_pose __eef_pose)
     (progn
       (if __eef_pose (setq _eef_pose (car __eef_pose)))
       _eef_pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _eef_pose
    (send _eef_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _eef_pose
       (send _eef_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _eef_pose
     (send _eef_pose :deserialize buf ptr-) (incf ptr- (send _eef_pose :serialization-length))
   ;;
   self)
  )

(defclass rokae_jps_navigation::eefState
  :super ros::object
  :slots ())

(setf (get rokae_jps_navigation::eefState :md5sum-) "dc45c501754cc9c6c2ec61a0d5b19682")
(setf (get rokae_jps_navigation::eefState :datatype-) "rokae_jps_navigation/eefState")
(setf (get rokae_jps_navigation::eefState :request) rokae_jps_navigation::eefStateRequest)
(setf (get rokae_jps_navigation::eefState :response) rokae_jps_navigation::eefStateResponse)

(defmethod rokae_jps_navigation::eefStateRequest
  (:response () (instance rokae_jps_navigation::eefStateResponse :init)))

(setf (get rokae_jps_navigation::eefStateRequest :md5sum-) "dc45c501754cc9c6c2ec61a0d5b19682")
(setf (get rokae_jps_navigation::eefStateRequest :datatype-) "rokae_jps_navigation/eefStateRequest")
(setf (get rokae_jps_navigation::eefStateRequest :definition-)
      "bool ifVerbose
---
geometry_msgs/Pose eef_pose


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

(setf (get rokae_jps_navigation::eefStateResponse :md5sum-) "dc45c501754cc9c6c2ec61a0d5b19682")
(setf (get rokae_jps_navigation::eefStateResponse :datatype-) "rokae_jps_navigation/eefStateResponse")
(setf (get rokae_jps_navigation::eefStateResponse :definition-)
      "bool ifVerbose
---
geometry_msgs/Pose eef_pose


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



(provide :rokae_jps_navigation/eefState "dc45c501754cc9c6c2ec61a0d5b19682")


