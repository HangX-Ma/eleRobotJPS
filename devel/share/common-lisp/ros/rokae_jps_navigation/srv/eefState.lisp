; Auto-generated. Do not edit!


(cl:in-package rokae_jps_navigation-srv)


;//! \htmlinclude eefState-request.msg.html

(cl:defclass <eefState-request> (roslisp-msg-protocol:ros-message)
  ((ifVerbose
    :reader ifVerbose
    :initarg :ifVerbose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass eefState-request (<eefState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eefState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eefState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<eefState-request> is deprecated: use rokae_jps_navigation-srv:eefState-request instead.")))

(cl:ensure-generic-function 'ifVerbose-val :lambda-list '(m))
(cl:defmethod ifVerbose-val ((m <eefState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:ifVerbose-val is deprecated.  Use rokae_jps_navigation-srv:ifVerbose instead.")
  (ifVerbose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eefState-request>) ostream)
  "Serializes a message object of type '<eefState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ifVerbose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eefState-request>) istream)
  "Deserializes a message object of type '<eefState-request>"
    (cl:setf (cl:slot-value msg 'ifVerbose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eefState-request>)))
  "Returns string type for a service object of type '<eefState-request>"
  "rokae_jps_navigation/eefStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eefState-request)))
  "Returns string type for a service object of type 'eefState-request"
  "rokae_jps_navigation/eefStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eefState-request>)))
  "Returns md5sum for a message object of type '<eefState-request>"
  "dc45c501754cc9c6c2ec61a0d5b19682")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eefState-request)))
  "Returns md5sum for a message object of type 'eefState-request"
  "dc45c501754cc9c6c2ec61a0d5b19682")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eefState-request>)))
  "Returns full string definition for message of type '<eefState-request>"
  (cl:format cl:nil "bool ifVerbose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eefState-request)))
  "Returns full string definition for message of type 'eefState-request"
  (cl:format cl:nil "bool ifVerbose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eefState-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eefState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'eefState-request
    (cl:cons ':ifVerbose (ifVerbose msg))
))
;//! \htmlinclude eefState-response.msg.html

(cl:defclass <eefState-response> (roslisp-msg-protocol:ros-message)
  ((eef_pose
    :reader eef_pose
    :initarg :eef_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass eefState-response (<eefState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eefState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eefState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<eefState-response> is deprecated: use rokae_jps_navigation-srv:eefState-response instead.")))

(cl:ensure-generic-function 'eef_pose-val :lambda-list '(m))
(cl:defmethod eef_pose-val ((m <eefState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:eef_pose-val is deprecated.  Use rokae_jps_navigation-srv:eef_pose instead.")
  (eef_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eefState-response>) ostream)
  "Serializes a message object of type '<eefState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'eef_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eefState-response>) istream)
  "Deserializes a message object of type '<eefState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'eef_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eefState-response>)))
  "Returns string type for a service object of type '<eefState-response>"
  "rokae_jps_navigation/eefStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eefState-response)))
  "Returns string type for a service object of type 'eefState-response"
  "rokae_jps_navigation/eefStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eefState-response>)))
  "Returns md5sum for a message object of type '<eefState-response>"
  "dc45c501754cc9c6c2ec61a0d5b19682")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eefState-response)))
  "Returns md5sum for a message object of type 'eefState-response"
  "dc45c501754cc9c6c2ec61a0d5b19682")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eefState-response>)))
  "Returns full string definition for message of type '<eefState-response>"
  (cl:format cl:nil "geometry_msgs/Pose eef_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eefState-response)))
  "Returns full string definition for message of type 'eefState-response"
  (cl:format cl:nil "geometry_msgs/Pose eef_pose~%~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eefState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'eef_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eefState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'eefState-response
    (cl:cons ':eef_pose (eef_pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'eefState)))
  'eefState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'eefState)))
  'eefState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eefState)))
  "Returns string type for a service object of type '<eefState>"
  "rokae_jps_navigation/eefState")