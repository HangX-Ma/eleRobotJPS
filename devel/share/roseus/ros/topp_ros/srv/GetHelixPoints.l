;; Auto-generated. Do not edit!


(when (boundp 'topp_ros::GetHelixPoints)
  (if (not (find-package "TOPP_ROS"))
    (make-package "TOPP_ROS"))
  (shadow 'GetHelixPoints (find-package "TOPP_ROS")))
(unless (find-package "TOPP_ROS::GETHELIXPOINTS")
  (make-package "TOPP_ROS::GETHELIXPOINTS"))
(unless (find-package "TOPP_ROS::GETHELIXPOINTSREQUEST")
  (make-package "TOPP_ROS::GETHELIXPOINTSREQUEST"))
(unless (find-package "TOPP_ROS::GETHELIXPOINTSRESPONSE")
  (make-package "TOPP_ROS::GETHELIXPOINTSRESPONSE"))

(in-package "ROS")



(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass topp_ros::GetHelixPointsRequest
  :super ros::object
  :slots (_r _angleStep _x0 _y0 _z0 _zf _deltaZ ))

(defmethod topp_ros::GetHelixPointsRequest
  (:init
   (&key
    ((:r __r) 0.0)
    ((:angleStep __angleStep) 0.0)
    ((:x0 __x0) 0.0)
    ((:y0 __y0) 0.0)
    ((:z0 __z0) 0.0)
    ((:zf __zf) 0.0)
    ((:deltaZ __deltaZ) 0.0)
    )
   (send-super :init)
   (setq _r (float __r))
   (setq _angleStep (float __angleStep))
   (setq _x0 (float __x0))
   (setq _y0 (float __y0))
   (setq _z0 (float __z0))
   (setq _zf (float __zf))
   (setq _deltaZ (float __deltaZ))
   self)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:angleStep
   (&optional __angleStep)
   (if __angleStep (setq _angleStep __angleStep)) _angleStep)
  (:x0
   (&optional __x0)
   (if __x0 (setq _x0 __x0)) _x0)
  (:y0
   (&optional __y0)
   (if __y0 (setq _y0 __y0)) _y0)
  (:z0
   (&optional __z0)
   (if __z0 (setq _z0 __z0)) _z0)
  (:zf
   (&optional __zf)
   (if __zf (setq _zf __zf)) _zf)
  (:deltaZ
   (&optional __deltaZ)
   (if __deltaZ (setq _deltaZ __deltaZ)) _deltaZ)
  (:serialization-length
   ()
   (+
    ;; float64 _r
    8
    ;; float64 _angleStep
    8
    ;; float64 _x0
    8
    ;; float64 _y0
    8
    ;; float64 _z0
    8
    ;; float64 _zf
    8
    ;; float64 _deltaZ
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angleStep
       (sys::poke _angleStep (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x0
       (sys::poke _x0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y0
       (sys::poke _y0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z0
       (sys::poke _z0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _zf
       (sys::poke _zf (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _deltaZ
       (sys::poke _deltaZ (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angleStep
     (setq _angleStep (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x0
     (setq _x0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y0
     (setq _y0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z0
     (setq _z0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _zf
     (setq _zf (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _deltaZ
     (setq _deltaZ (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass topp_ros::GetHelixPointsResponse
  :super ros::object
  :slots (_helix_points ))

(defmethod topp_ros::GetHelixPointsResponse
  (:init
   (&key
    ((:helix_points __helix_points) (instance trajectory_msgs::JointTrajectory :init))
    )
   (send-super :init)
   (setq _helix_points __helix_points)
   self)
  (:helix_points
   (&rest __helix_points)
   (if (keywordp (car __helix_points))
       (send* _helix_points __helix_points)
     (progn
       (if __helix_points (setq _helix_points (car __helix_points)))
       _helix_points)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _helix_points
    (send _helix_points :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _helix_points
       (send _helix_points :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _helix_points
     (send _helix_points :deserialize buf ptr-) (incf ptr- (send _helix_points :serialization-length))
   ;;
   self)
  )

(defclass topp_ros::GetHelixPoints
  :super ros::object
  :slots ())

(setf (get topp_ros::GetHelixPoints :md5sum-) "822f91513c2ae98aef72e6642f42233f")
(setf (get topp_ros::GetHelixPoints :datatype-) "topp_ros/GetHelixPoints")
(setf (get topp_ros::GetHelixPoints :request) topp_ros::GetHelixPointsRequest)
(setf (get topp_ros::GetHelixPoints :response) topp_ros::GetHelixPointsResponse)

(defmethod topp_ros::GetHelixPointsRequest
  (:response () (instance topp_ros::GetHelixPointsResponse :init)))

(setf (get topp_ros::GetHelixPointsRequest :md5sum-) "822f91513c2ae98aef72e6642f42233f")
(setf (get topp_ros::GetHelixPointsRequest :datatype-) "topp_ros/GetHelixPointsRequest")
(setf (get topp_ros::GetHelixPointsRequest :definition-)
      "float64 r
float64 angleStep
float64 x0
float64 y0
float64 z0
float64 zf
float64 deltaZ
---
trajectory_msgs/JointTrajectory helix_points

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

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

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")

(setf (get topp_ros::GetHelixPointsResponse :md5sum-) "822f91513c2ae98aef72e6642f42233f")
(setf (get topp_ros::GetHelixPointsResponse :datatype-) "topp_ros/GetHelixPointsResponse")
(setf (get topp_ros::GetHelixPointsResponse :definition-)
      "float64 r
float64 angleStep
float64 x0
float64 y0
float64 z0
float64 zf
float64 deltaZ
---
trajectory_msgs/JointTrajectory helix_points

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

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

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
")



(provide :topp_ros/GetHelixPoints "822f91513c2ae98aef72e6642f42233f")


