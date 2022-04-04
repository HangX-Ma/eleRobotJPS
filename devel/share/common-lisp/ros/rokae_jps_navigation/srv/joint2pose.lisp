; Auto-generated. Do not edit!


(cl:in-package rokae_jps_navigation-srv)


;//! \htmlinclude joint2pose-request.msg.html

(cl:defclass <joint2pose-request> (roslisp-msg-protocol:ros-message)
  ((joint0
    :reader joint0
    :initarg :joint0
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (joint1
    :reader joint1
    :initarg :joint1
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (joint2
    :reader joint2
    :initarg :joint2
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (joint3
    :reader joint3
    :initarg :joint3
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (joint4
    :reader joint4
    :initarg :joint4
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (joint5
    :reader joint5
    :initarg :joint5
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint3) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint4) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint5) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ifVerbose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint2pose-request>) istream)
  "Deserializes a message object of type '<joint2pose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint3) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint4) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint5) istream)
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
  "5999e34ffaace7b38734bfad5de479e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint2pose-request)))
  "Returns md5sum for a message object of type 'joint2pose-request"
  "5999e34ffaace7b38734bfad5de479e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint2pose-request>)))
  "Returns full string definition for message of type '<joint2pose-request>"
  (cl:format cl:nil "std_msgs/Float32 joint0~%std_msgs/Float32 joint1~%std_msgs/Float32 joint2~%std_msgs/Float32 joint3~%std_msgs/Float32 joint4~%std_msgs/Float32 joint5~%bool ifVerbose~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint2pose-request)))
  "Returns full string definition for message of type 'joint2pose-request"
  (cl:format cl:nil "std_msgs/Float32 joint0~%std_msgs/Float32 joint1~%std_msgs/Float32 joint2~%std_msgs/Float32 joint3~%std_msgs/Float32 joint4~%std_msgs/Float32 joint5~%bool ifVerbose~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint2pose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint3))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint4))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint5))
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
  ()
)

(cl:defclass joint2pose-response (<joint2pose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joint2pose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joint2pose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<joint2pose-response> is deprecated: use rokae_jps_navigation-srv:joint2pose-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joint2pose-response>) ostream)
  "Serializes a message object of type '<joint2pose-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joint2pose-response>) istream)
  "Deserializes a message object of type '<joint2pose-response>"
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
  "5999e34ffaace7b38734bfad5de479e9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joint2pose-response)))
  "Returns md5sum for a message object of type 'joint2pose-response"
  "5999e34ffaace7b38734bfad5de479e9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joint2pose-response>)))
  "Returns full string definition for message of type '<joint2pose-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joint2pose-response)))
  "Returns full string definition for message of type 'joint2pose-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joint2pose-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joint2pose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'joint2pose-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'joint2pose)))
  'joint2pose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'joint2pose)))
  'joint2pose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joint2pose)))
  "Returns string type for a service object of type '<joint2pose>"
  "rokae_jps_navigation/joint2pose")