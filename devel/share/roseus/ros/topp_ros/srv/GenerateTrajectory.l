;; Auto-generated. Do not edit!


(when (boundp 'topp_ros::GenerateTrajectory)
  (if (not (find-package "TOPP_ROS"))
    (make-package "TOPP_ROS"))
  (shadow 'GenerateTrajectory (find-package "TOPP_ROS")))
(unless (find-package "TOPP_ROS::GENERATETRAJECTORY")
  (make-package "TOPP_ROS::GENERATETRAJECTORY"))
(unless (find-package "TOPP_ROS::GENERATETRAJECTORYREQUEST")
  (make-package "TOPP_ROS::GENERATETRAJECTORYREQUEST"))
(unless (find-package "TOPP_ROS::GENERATETRAJECTORYRESPONSE")
  (make-package "TOPP_ROS::GENERATETRAJECTORYRESPONSE"))

(in-package "ROS")

(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))


(defclass topp_ros::GenerateTrajectoryRequest
  :super ros::object
  :slots (_waypoints _sampling_frequency _n_gridpoints _plot ))

(defmethod topp_ros::GenerateTrajectoryRequest
  (:init
   (&key
    ((:waypoints __waypoints) (instance trajectory_msgs::JointTrajectory :init))
    ((:sampling_frequency __sampling_frequency) 0.0)
    ((:n_gridpoints __n_gridpoints) 0)
    ((:plot __plot) nil)
    )
   (send-super :init)
   (setq _waypoints __waypoints)
   (setq _sampling_frequency (float __sampling_frequency))
   (setq _n_gridpoints (round __n_gridpoints))
   (setq _plot __plot)
   self)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:sampling_frequency
   (&optional __sampling_frequency)
   (if __sampling_frequency (setq _sampling_frequency __sampling_frequency)) _sampling_frequency)
  (:n_gridpoints
   (&optional __n_gridpoints)
   (if __n_gridpoints (setq _n_gridpoints __n_gridpoints)) _n_gridpoints)
  (:plot
   (&optional __plot)
   (if __plot (setq _plot __plot)) _plot)
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _waypoints
    (send _waypoints :serialization-length)
    ;; float64 _sampling_frequency
    8
    ;; int32 _n_gridpoints
    4
    ;; bool _plot
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _waypoints
       (send _waypoints :serialize s)
     ;; float64 _sampling_frequency
       (sys::poke _sampling_frequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _n_gridpoints
       (write-long _n_gridpoints s)
     ;; bool _plot
       (if _plot (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _waypoints
     (send _waypoints :deserialize buf ptr-) (incf ptr- (send _waypoints :serialization-length))
   ;; float64 _sampling_frequency
     (setq _sampling_frequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _n_gridpoints
     (setq _n_gridpoints (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _plot
     (setq _plot (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass topp_ros::GenerateTrajectoryResponse
  :super ros::object
  :slots (_trajectory _success ))

(defmethod topp_ros::GenerateTrajectoryResponse
  (:init
   (&key
    ((:trajectory __trajectory) (instance trajectory_msgs::JointTrajectory :init))
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _trajectory __trajectory)
   (setq _success __success)
   self)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _trajectory
    (send _trajectory :serialization-length)
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _trajectory
       (send _trajectory :serialize s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass topp_ros::GenerateTrajectory
  :super ros::object
  :slots ())

(setf (get topp_ros::GenerateTrajectory :md5sum-) "bf5d8c7f17b854de72ace6f948a3f238")
(setf (get topp_ros::GenerateTrajectory :datatype-) "topp_ros/GenerateTrajectory")
(setf (get topp_ros::GenerateTrajectory :request) topp_ros::GenerateTrajectoryRequest)
(setf (get topp_ros::GenerateTrajectory :response) topp_ros::GenerateTrajectoryResponse)

(defmethod topp_ros::GenerateTrajectoryRequest
  (:response () (instance topp_ros::GenerateTrajectoryResponse :init)))

(setf (get topp_ros::GenerateTrajectoryRequest :md5sum-) "bf5d8c7f17b854de72ace6f948a3f238")
(setf (get topp_ros::GenerateTrajectoryRequest :datatype-) "topp_ros/GenerateTrajectoryRequest")
(setf (get topp_ros::GenerateTrajectoryRequest :definition-)
      "trajectory_msgs/JointTrajectory     waypoints
float64                             sampling_frequency
int32                               n_gridpoints
bool                                plot

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
---
trajectory_msgs/JointTrajectory     trajectory
bool                                success


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

(setf (get topp_ros::GenerateTrajectoryResponse :md5sum-) "bf5d8c7f17b854de72ace6f948a3f238")
(setf (get topp_ros::GenerateTrajectoryResponse :datatype-) "topp_ros/GenerateTrajectoryResponse")
(setf (get topp_ros::GenerateTrajectoryResponse :definition-)
      "trajectory_msgs/JointTrajectory     waypoints
float64                             sampling_frequency
int32                               n_gridpoints
bool                                plot

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
---
trajectory_msgs/JointTrajectory     trajectory
bool                                success


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



(provide :topp_ros/GenerateTrajectory "bf5d8c7f17b854de72ace6f948a3f238")


