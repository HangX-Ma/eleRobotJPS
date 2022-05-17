; Auto-generated. Do not edit!


(cl:in-package topp_ros-srv)


;//! \htmlinclude GetHelixPoints-request.msg.html

(cl:defclass <GetHelixPoints-request> (roslisp-msg-protocol:ros-message)
  ((r
    :reader r
    :initarg :r
    :type cl:float
    :initform 0.0)
   (angleStep
    :reader angleStep
    :initarg :angleStep
    :type cl:float
    :initform 0.0)
   (x0
    :reader x0
    :initarg :x0
    :type cl:float
    :initform 0.0)
   (y0
    :reader y0
    :initarg :y0
    :type cl:float
    :initform 0.0)
   (z0
    :reader z0
    :initarg :z0
    :type cl:float
    :initform 0.0)
   (zf
    :reader zf
    :initarg :zf
    :type cl:float
    :initform 0.0)
   (deltaZ
    :reader deltaZ
    :initarg :deltaZ
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetHelixPoints-request (<GetHelixPoints-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHelixPoints-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHelixPoints-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name topp_ros-srv:<GetHelixPoints-request> is deprecated: use topp_ros-srv:GetHelixPoints-request instead.")))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:r-val is deprecated.  Use topp_ros-srv:r instead.")
  (r m))

(cl:ensure-generic-function 'angleStep-val :lambda-list '(m))
(cl:defmethod angleStep-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:angleStep-val is deprecated.  Use topp_ros-srv:angleStep instead.")
  (angleStep m))

(cl:ensure-generic-function 'x0-val :lambda-list '(m))
(cl:defmethod x0-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:x0-val is deprecated.  Use topp_ros-srv:x0 instead.")
  (x0 m))

(cl:ensure-generic-function 'y0-val :lambda-list '(m))
(cl:defmethod y0-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:y0-val is deprecated.  Use topp_ros-srv:y0 instead.")
  (y0 m))

(cl:ensure-generic-function 'z0-val :lambda-list '(m))
(cl:defmethod z0-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:z0-val is deprecated.  Use topp_ros-srv:z0 instead.")
  (z0 m))

(cl:ensure-generic-function 'zf-val :lambda-list '(m))
(cl:defmethod zf-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:zf-val is deprecated.  Use topp_ros-srv:zf instead.")
  (zf m))

(cl:ensure-generic-function 'deltaZ-val :lambda-list '(m))
(cl:defmethod deltaZ-val ((m <GetHelixPoints-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:deltaZ-val is deprecated.  Use topp_ros-srv:deltaZ instead.")
  (deltaZ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHelixPoints-request>) ostream)
  "Serializes a message object of type '<GetHelixPoints-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angleStep))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'zf))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'deltaZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHelixPoints-request>) istream)
  "Deserializes a message object of type '<GetHelixPoints-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angleStep) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'zf) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'deltaZ) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHelixPoints-request>)))
  "Returns string type for a service object of type '<GetHelixPoints-request>"
  "topp_ros/GetHelixPointsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHelixPoints-request)))
  "Returns string type for a service object of type 'GetHelixPoints-request"
  "topp_ros/GetHelixPointsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHelixPoints-request>)))
  "Returns md5sum for a message object of type '<GetHelixPoints-request>"
  "822f91513c2ae98aef72e6642f42233f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHelixPoints-request)))
  "Returns md5sum for a message object of type 'GetHelixPoints-request"
  "822f91513c2ae98aef72e6642f42233f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHelixPoints-request>)))
  "Returns full string definition for message of type '<GetHelixPoints-request>"
  (cl:format cl:nil "float64 r~%float64 angleStep~%float64 x0~%float64 y0~%float64 z0~%float64 zf~%float64 deltaZ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHelixPoints-request)))
  "Returns full string definition for message of type 'GetHelixPoints-request"
  (cl:format cl:nil "float64 r~%float64 angleStep~%float64 x0~%float64 y0~%float64 z0~%float64 zf~%float64 deltaZ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHelixPoints-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHelixPoints-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHelixPoints-request
    (cl:cons ':r (r msg))
    (cl:cons ':angleStep (angleStep msg))
    (cl:cons ':x0 (x0 msg))
    (cl:cons ':y0 (y0 msg))
    (cl:cons ':z0 (z0 msg))
    (cl:cons ':zf (zf msg))
    (cl:cons ':deltaZ (deltaZ msg))
))
;//! \htmlinclude GetHelixPoints-response.msg.html

(cl:defclass <GetHelixPoints-response> (roslisp-msg-protocol:ros-message)
  ((helix_points
    :reader helix_points
    :initarg :helix_points
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass GetHelixPoints-response (<GetHelixPoints-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetHelixPoints-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetHelixPoints-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name topp_ros-srv:<GetHelixPoints-response> is deprecated: use topp_ros-srv:GetHelixPoints-response instead.")))

(cl:ensure-generic-function 'helix_points-val :lambda-list '(m))
(cl:defmethod helix_points-val ((m <GetHelixPoints-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader topp_ros-srv:helix_points-val is deprecated.  Use topp_ros-srv:helix_points instead.")
  (helix_points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetHelixPoints-response>) ostream)
  "Serializes a message object of type '<GetHelixPoints-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'helix_points) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetHelixPoints-response>) istream)
  "Deserializes a message object of type '<GetHelixPoints-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'helix_points) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetHelixPoints-response>)))
  "Returns string type for a service object of type '<GetHelixPoints-response>"
  "topp_ros/GetHelixPointsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHelixPoints-response)))
  "Returns string type for a service object of type 'GetHelixPoints-response"
  "topp_ros/GetHelixPointsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetHelixPoints-response>)))
  "Returns md5sum for a message object of type '<GetHelixPoints-response>"
  "822f91513c2ae98aef72e6642f42233f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetHelixPoints-response)))
  "Returns md5sum for a message object of type 'GetHelixPoints-response"
  "822f91513c2ae98aef72e6642f42233f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetHelixPoints-response>)))
  "Returns full string definition for message of type '<GetHelixPoints-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory helix_points~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetHelixPoints-response)))
  "Returns full string definition for message of type 'GetHelixPoints-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory helix_points~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetHelixPoints-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'helix_points))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetHelixPoints-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetHelixPoints-response
    (cl:cons ':helix_points (helix_points msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetHelixPoints)))
  'GetHelixPoints-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetHelixPoints)))
  'GetHelixPoints-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetHelixPoints)))
  "Returns string type for a service object of type '<GetHelixPoints>"
  "topp_ros/GetHelixPoints")