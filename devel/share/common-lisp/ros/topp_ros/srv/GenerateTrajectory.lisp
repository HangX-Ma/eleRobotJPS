; Auto-generated. Do not edit!


(cl:in-package topp_ros-srv)


;//! \htmlinclude GenerateTrajectory-request.msg.html

(cl:defclass <GenerateTrajectory-request> (roslisp-msg-protocol:ros-message)
  ((waypoints
    :reader waypoints
    :initarg :waypoints
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory))
   (sampling_frequency
    :reader sampling_frequency
    :initarg :sampling_frequency
    :type cl:float
    :initform 0.0)
   (n_gridpoints
    :reader n_gridpoints
    :initarg :n_gridpoints
    :type cl:integer
    :initform 0)
   (plot
    :reader plot
    :initarg :plot
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GenerateTrajectory-request (<GenerateTrajectory-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateTrajectory-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateTrajectory-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name topp_ros-srv:<GenerateTrajectory-request> is deprecated: use topp_ros-srv:GenerateTrajectory-request instead.")))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:waypoints-val is deprecated.  Use topp_ros-srv:waypoints instead.")
  (waypoints m))

(cl:ensure-generic-function 'sampling_frequency-val :lambda-list '(m))
(cl:defmethod sampling_frequency-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:sampling_frequency-val is deprecated.  Use topp_ros-srv:sampling_frequency instead.")
  (sampling_frequency m))

(cl:ensure-generic-function 'n_gridpoints-val :lambda-list '(m))
(cl:defmethod n_gridpoints-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:n_gridpoints-val is deprecated.  Use topp_ros-srv:n_gridpoints instead.")
  (n_gridpoints m))

(cl:ensure-generic-function 'plot-val :lambda-list '(m))
(cl:defmethod plot-val ((m <GenerateTrajectory-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:plot-val is deprecated.  Use topp_ros-srv:plot instead.")
  (plot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateTrajectory-request>) ostream)
  "Serializes a message object of type '<GenerateTrajectory-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoints) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'sampling_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'n_gridpoints)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'plot) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateTrajectory-request>) istream)
  "Deserializes a message object of type '<GenerateTrajectory-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoints) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sampling_frequency) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_gridpoints) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'plot) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateTrajectory-request>)))
  "Returns string type for a service object of type '<GenerateTrajectory-request>"
  "topp_ros/GenerateTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateTrajectory-request)))
  "Returns string type for a service object of type 'GenerateTrajectory-request"
  "topp_ros/GenerateTrajectoryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateTrajectory-request>)))
  "Returns md5sum for a message object of type '<GenerateTrajectory-request>"
  "bf5d8c7f17b854de72ace6f948a3f238")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateTrajectory-request)))
  "Returns md5sum for a message object of type 'GenerateTrajectory-request"
  "bf5d8c7f17b854de72ace6f948a3f238")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateTrajectory-request>)))
  "Returns full string definition for message of type '<GenerateTrajectory-request>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory     waypoints~%float64                             sampling_frequency~%int32                               n_gridpoints~%bool                                plot~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateTrajectory-request)))
  "Returns full string definition for message of type 'GenerateTrajectory-request"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory     waypoints~%float64                             sampling_frequency~%int32                               n_gridpoints~%bool                                plot~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateTrajectory-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoints))
     8
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateTrajectory-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateTrajectory-request
    (cl:cons ':waypoints (waypoints msg))
    (cl:cons ':sampling_frequency (sampling_frequency msg))
    (cl:cons ':n_gridpoints (n_gridpoints msg))
    (cl:cons ':plot (plot msg))
))
;//! \htmlinclude GenerateTrajectory-response.msg.html

(cl:defclass <GenerateTrajectory-response> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GenerateTrajectory-response (<GenerateTrajectory-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GenerateTrajectory-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GenerateTrajectory-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name topp_ros-srv:<GenerateTrajectory-response> is deprecated: use topp_ros-srv:GenerateTrajectory-response instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <GenerateTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:trajectory-val is deprecated.  Use topp_ros-srv:trajectory instead.")
  (trajectory m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GenerateTrajectory-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:success-val is deprecated.  Use topp_ros-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GenerateTrajectory-response>) ostream)
  "Serializes a message object of type '<GenerateTrajectory-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GenerateTrajectory-response>) istream)
  "Deserializes a message object of type '<GenerateTrajectory-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GenerateTrajectory-response>)))
  "Returns string type for a service object of type '<GenerateTrajectory-response>"
  "topp_ros/GenerateTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateTrajectory-response)))
  "Returns string type for a service object of type 'GenerateTrajectory-response"
  "topp_ros/GenerateTrajectoryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GenerateTrajectory-response>)))
  "Returns md5sum for a message object of type '<GenerateTrajectory-response>"
  "bf5d8c7f17b854de72ace6f948a3f238")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GenerateTrajectory-response)))
  "Returns md5sum for a message object of type 'GenerateTrajectory-response"
  "bf5d8c7f17b854de72ace6f948a3f238")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GenerateTrajectory-response>)))
  "Returns full string definition for message of type '<GenerateTrajectory-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory     trajectory~%bool                                success~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GenerateTrajectory-response)))
  "Returns full string definition for message of type 'GenerateTrajectory-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory     trajectory~%bool                                success~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GenerateTrajectory-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GenerateTrajectory-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GenerateTrajectory-response
    (cl:cons ':trajectory (trajectory msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GenerateTrajectory)))
  'GenerateTrajectory-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GenerateTrajectory)))
  'GenerateTrajectory-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GenerateTrajectory)))
  "Returns string type for a service object of type '<GenerateTrajectory>"
  "topp_ros/GenerateTrajectory")