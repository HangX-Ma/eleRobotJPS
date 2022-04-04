; Auto-generated. Do not edit!


(cl:in-package rokae_jps_navigation-srv)


;//! \htmlinclude CheckCollision-request.msg.html

(cl:defclass <CheckCollision-request> (roslisp-msg-protocol:ros-message)
  ((path_pose
    :reader path_pose
    :initarg :path_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (prev_joints
    :reader prev_joints
    :initarg :prev_joints
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (ifVerbose
    :reader ifVerbose
    :initarg :ifVerbose
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckCollision-request (<CheckCollision-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckCollision-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckCollision-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<CheckCollision-request> is deprecated: use rokae_jps_navigation-srv:CheckCollision-request instead.")))

(cl:ensure-generic-function 'path_pose-val :lambda-list '(m))
(cl:defmethod path_pose-val ((m <CheckCollision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:path_pose-val is deprecated.  Use rokae_jps_navigation-srv:path_pose instead.")
  (path_pose m))

(cl:ensure-generic-function 'prev_joints-val :lambda-list '(m))
(cl:defmethod prev_joints-val ((m <CheckCollision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:prev_joints-val is deprecated.  Use rokae_jps_navigation-srv:prev_joints instead.")
  (prev_joints m))

(cl:ensure-generic-function 'ifVerbose-val :lambda-list '(m))
(cl:defmethod ifVerbose-val ((m <CheckCollision-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:ifVerbose-val is deprecated.  Use rokae_jps_navigation-srv:ifVerbose instead.")
  (ifVerbose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckCollision-request>) ostream)
  "Serializes a message object of type '<CheckCollision-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path_pose) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'prev_joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'prev_joints))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ifVerbose) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckCollision-request>) istream)
  "Deserializes a message object of type '<CheckCollision-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path_pose) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'prev_joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'prev_joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'ifVerbose) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckCollision-request>)))
  "Returns string type for a service object of type '<CheckCollision-request>"
  "rokae_jps_navigation/CheckCollisionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckCollision-request)))
  "Returns string type for a service object of type 'CheckCollision-request"
  "rokae_jps_navigation/CheckCollisionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckCollision-request>)))
  "Returns md5sum for a message object of type '<CheckCollision-request>"
  "9553736970fcba3d957b6a5220bb6872")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckCollision-request)))
  "Returns md5sum for a message object of type 'CheckCollision-request"
  "9553736970fcba3d957b6a5220bb6872")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckCollision-request>)))
  "Returns full string definition for message of type '<CheckCollision-request>"
  (cl:format cl:nil "geometry_msgs/Pose path_pose~%float32[] prev_joints~%bool ifVerbose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckCollision-request)))
  "Returns full string definition for message of type 'CheckCollision-request"
  (cl:format cl:nil "geometry_msgs/Pose path_pose~%float32[] prev_joints~%bool ifVerbose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckCollision-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path_pose))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'prev_joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckCollision-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckCollision-request
    (cl:cons ':path_pose (path_pose msg))
    (cl:cons ':prev_joints (prev_joints msg))
    (cl:cons ':ifVerbose (ifVerbose msg))
))
;//! \htmlinclude CheckCollision-response.msg.html

(cl:defclass <CheckCollision-response> (roslisp-msg-protocol:ros-message)
  ((isCollide
    :reader isCollide
    :initarg :isCollide
    :type cl:boolean
    :initform cl:nil)
   (getSolution
    :reader getSolution
    :initarg :getSolution
    :type cl:boolean
    :initform cl:nil)
   (curr_joints
    :reader curr_joints
    :initarg :curr_joints
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CheckCollision-response (<CheckCollision-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckCollision-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckCollision-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_jps_navigation-srv:<CheckCollision-response> is deprecated: use rokae_jps_navigation-srv:CheckCollision-response instead.")))

(cl:ensure-generic-function 'isCollide-val :lambda-list '(m))
(cl:defmethod isCollide-val ((m <CheckCollision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:isCollide-val is deprecated.  Use rokae_jps_navigation-srv:isCollide instead.")
  (isCollide m))

(cl:ensure-generic-function 'getSolution-val :lambda-list '(m))
(cl:defmethod getSolution-val ((m <CheckCollision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:getSolution-val is deprecated.  Use rokae_jps_navigation-srv:getSolution instead.")
  (getSolution m))

(cl:ensure-generic-function 'curr_joints-val :lambda-list '(m))
(cl:defmethod curr_joints-val ((m <CheckCollision-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_jps_navigation-srv:curr_joints-val is deprecated.  Use rokae_jps_navigation-srv:curr_joints instead.")
  (curr_joints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckCollision-response>) ostream)
  "Serializes a message object of type '<CheckCollision-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isCollide) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'getSolution) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'curr_joints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'curr_joints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckCollision-response>) istream)
  "Deserializes a message object of type '<CheckCollision-response>"
    (cl:setf (cl:slot-value msg 'isCollide) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'getSolution) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'curr_joints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'curr_joints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckCollision-response>)))
  "Returns string type for a service object of type '<CheckCollision-response>"
  "rokae_jps_navigation/CheckCollisionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckCollision-response)))
  "Returns string type for a service object of type 'CheckCollision-response"
  "rokae_jps_navigation/CheckCollisionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckCollision-response>)))
  "Returns md5sum for a message object of type '<CheckCollision-response>"
  "9553736970fcba3d957b6a5220bb6872")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckCollision-response)))
  "Returns md5sum for a message object of type 'CheckCollision-response"
  "9553736970fcba3d957b6a5220bb6872")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckCollision-response>)))
  "Returns full string definition for message of type '<CheckCollision-response>"
  (cl:format cl:nil "bool isCollide~%bool getSolution~%float32[] curr_joints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckCollision-response)))
  "Returns full string definition for message of type 'CheckCollision-response"
  (cl:format cl:nil "bool isCollide~%bool getSolution~%float32[] curr_joints~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckCollision-response>))
  (cl:+ 0
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'curr_joints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckCollision-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckCollision-response
    (cl:cons ':isCollide (isCollide msg))
    (cl:cons ':getSolution (getSolution msg))
    (cl:cons ':curr_joints (curr_joints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckCollision)))
  'CheckCollision-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckCollision)))
  'CheckCollision-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckCollision)))
  "Returns string type for a service object of type '<CheckCollision>"
  "rokae_jps_navigation/CheckCollision")