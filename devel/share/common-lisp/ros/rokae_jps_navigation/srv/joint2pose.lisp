; Auto-generated. Do not edit!


(cl:in-package rokae_jps_navigation-srv)


;//! \htmlinclude joint2pose-request.msg.html

(cl:defclass <joint2pose-request> (roslisp-msg-protocol:ros-message)
  ((joint0
    :reader joint0
    :initarg :joint0
    :type cl:float
    :initform 0.0)
   (joint1
    :reader joint1
    :initarg :joint1
    :type cl:float
    :initform 0.0)
   (joint2
    :reader joint2
    :initarg :joint2
    :type cl:float
    :initform 0.0)
   (joint3
    :reader joint3
    :initarg :joint3
    :type cl:float
    :initform 0.0)
   (joint4
    :reader joint4
    :initarg :joint4
    :type cl:float
    :initform 0.0)
   (joint5
    :reader joint5
    :initarg :joint5
    :type cl:float
    :initform 0.0)
   (ifVerbose
    :reader ifVerbose
    :initarg :ifVerbose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass joint2pose-request (<joint2pose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint2pose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint2pose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<joint2pose-request> is deprecated: use rokae_jps_navigation-srv:joint2pose-request instead.")))

(cl:ensure-generic-function 'joint0-val :lambda-list '(m))
(cl:defmethod joint0-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:joint0-val is deprecated.  Use rokae_jps_navigation-srv:joint0 instead.")
  (joint0 m))

(cl:ensure-generic-function 'joint1-val :lambda-list '(m))
(cl:defmethod joint1-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:joint1-val is deprecated.  Use rokae_jps_navigation-srv:joint1 instead.")
  (joint1 m))

(cl:ensure-generic-function 'joint2-val :lambda-list '(m))
(cl:defmethod joint2-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:joint2-val is deprecated.  Use rokae_jps_navigation-srv:joint2 instead.")
  (joint2 m))

(cl:ensure-generic-function 'joint3-val :lambda-list '(m))
(cl:defmethod joint3-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:joint3-val is deprecated.  Use rokae_jps_navigation-srv:joint3 instead.")
  (joint3 m))

(cl:ensure-generic-function 'joint4-val :lambda-list '(m))
(cl:defmethod joint4-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:joint4-val is deprecated.  Use rokae_jps_navigation-srv:joint4 instead.")
  (joint4 m))

(cl:ensure-generic-function 'joint5-val :lambda-list '(m))
(cl:defmethod joint5-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:joint5-val is deprecated.  Use rokae_jps_navigation-srv:joint5 instead.")
  (joint5 m))

(cl:ensure-generic-function 'ifVerbose-val :lambda-list '(m))
(cl:defmethod ifVerbose-val ((m <joint2pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:ifVerbose-val is deprecated.  Use rokae_jps_navigation-srv:ifVerbose instead.")
  (ifVerbose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint2pose-request>) ostream)
  "Serializes a message object of type '<joint2pose-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'joint5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ifVerbose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint2pose-request>) istream)
  "Deserializes a message object of type '<joint2pose-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint4) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'joint5) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'ifVerbose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint2pose-request>)))
  "Returns string type for a service object of type '<joint2pose-request>"
  "rokae_jps_navigation/joint2poseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint2pose-request)))
  "Returns string type for a service object of type 'joint2pose-request"
  "rokae_jps_navigation/joint2poseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint2pose-request>)))
  "Returns md5sum for a message object of type '<joint2pose-request>"
  "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint2pose-request)))
  "Returns md5sum for a message object of type 'joint2pose-request"
  "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint2pose-request>)))
  "Returns full string definition for message of type '<joint2pose-request>"
  (cl:format cl:nil "float64 joint0~%float64 joint1~%float64 joint2~%float64 joint3~%float64 joint4~%float64 joint5~%bool ifVerbose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint2pose-request)))
  "Returns full string definition for message of type 'joint2pose-request"
  (cl:format cl:nil "float64 joint0~%float64 joint1~%float64 joint2~%float64 joint3~%float64 joint4~%float64 joint5~%bool ifVerbose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint2pose-request>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint2pose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'joint2pose-request
    (cl:cons ':joint0 (joint0 msg))
    (cl:cons ':joint1 (joint1 msg))
    (cl:cons ':joint2 (joint2 msg))
    (cl:cons ':joint3 (joint3 msg))
    (cl:cons ':joint4 (joint4 msg))
    (cl:cons ':joint5 (joint5 msg))
    (cl:cons ':ifVerbose (ifVerbose msg))
))
;//! \htmlinclude joint2pose-response.msg.html

(cl:defclass <joint2pose-response> (roslisp-msg-protocol:ros-message)
  ((re_pose
    :reader re_pose
    :initarg :re_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass joint2pose-response (<joint2pose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint2pose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint2pose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<joint2pose-response> is deprecated: use rokae_jps_navigation-srv:joint2pose-response instead.")))

(cl:ensure-generic-function 're_pose-val :lambda-list '(m))
(cl:defmethod re_pose-val ((m <joint2pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:re_pose-val is deprecated.  Use rokae_jps_navigation-srv:re_pose instead.")
  (re_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint2pose-response>) ostream)
  "Serializes a message object of type '<joint2pose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 're_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint2pose-response>) istream)
  "Deserializes a message object of type '<joint2pose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 're_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joint2pose-response>)))
  "Returns string type for a service object of type '<joint2pose-response>"
  "rokae_jps_navigation/joint2poseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint2pose-response)))
  "Returns string type for a service object of type 'joint2pose-response"
  "rokae_jps_navigation/joint2poseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joint2pose-response>)))
  "Returns md5sum for a message object of type '<joint2pose-response>"
  "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint2pose-response)))
  "Returns md5sum for a message object of type 'joint2pose-response"
  "8e3735eb3bfe1ca9ba91861cd4e5d3f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint2pose-response>)))
  "Returns full string definition for message of type '<joint2pose-response>"
  (cl:format cl:nil "geometry_msgs/Pose re_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint2pose-response)))
  "Returns full string definition for message of type 'joint2pose-response"
  (cl:format cl:nil "geometry_msgs/Pose re_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint2pose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 're_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint2pose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'joint2pose-response
    (cl:cons ':re_pose (re_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'joint2pose)))
  'joint2pose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'joint2pose)))
  'joint2pose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint2pose)))
  "Returns string type for a service object of type '<joint2pose>"
  "rokae_jps_navigation/joint2pose")