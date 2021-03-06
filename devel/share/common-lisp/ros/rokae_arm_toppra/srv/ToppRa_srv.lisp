; Auto-generated. Do not edit!


(cl:in-package rokae_arm_toppra-srv)


;//! \htmlinclude ToppRa_srv-request.msg.html

(cl:defclass <ToppRa_srv-request> (roslisp-msg-protocol:ros-message)
  ((joint_configs_on_way
    :reader joint_configs_on_way
    :initarg :joint_configs_on_way
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (ifSave
    :reader ifSave
    :initarg :ifSave
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ToppRa_srv-request (<ToppRa_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToppRa_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToppRa_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_arm_toppra-srv:<ToppRa_srv-request> is deprecated: use rokae_arm_toppra-srv:ToppRa_srv-request instead.")))

(cl:ensure-generic-function 'joint_configs_on_way-val :lambda-list '(m))
(cl:defmethod joint_configs_on_way-val ((m <ToppRa_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_arm_toppra-srv:joint_configs_on_way-val is deprecated.  Use rokae_arm_toppra-srv:joint_configs_on_way instead.")
  (joint_configs_on_way m))

(cl:ensure-generic-function 'ifSave-val :lambda-list '(m))
(cl:defmethod ifSave-val ((m <ToppRa_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_arm_toppra-srv:ifSave-val is deprecated.  Use rokae_arm_toppra-srv:ifSave instead.")
  (ifSave m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToppRa_srv-request>) ostream)
  "Serializes a message object of type '<ToppRa_srv-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_configs_on_way))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_configs_on_way))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ifSave) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToppRa_srv-request>) istream)
  "Deserializes a message object of type '<ToppRa_srv-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_configs_on_way) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_configs_on_way)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:slot-value msg 'ifSave) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToppRa_srv-request>)))
  "Returns string type for a service object of type '<ToppRa_srv-request>"
  "rokae_arm_toppra/ToppRa_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToppRa_srv-request)))
  "Returns string type for a service object of type 'ToppRa_srv-request"
  "rokae_arm_toppra/ToppRa_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToppRa_srv-request>)))
  "Returns md5sum for a message object of type '<ToppRa_srv-request>"
  "7cd293dc242907880f3bae1412ae9b57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToppRa_srv-request)))
  "Returns md5sum for a message object of type 'ToppRa_srv-request"
  "7cd293dc242907880f3bae1412ae9b57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToppRa_srv-request>)))
  "Returns full string definition for message of type '<ToppRa_srv-request>"
  (cl:format cl:nil "float64[] joint_configs_on_way ~%bool ifSave~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToppRa_srv-request)))
  "Returns full string definition for message of type 'ToppRa_srv-request"
  (cl:format cl:nil "float64[] joint_configs_on_way ~%bool ifSave~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToppRa_srv-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_configs_on_way) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToppRa_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ToppRa_srv-request
    (cl:cons ':joint_configs_on_way (joint_configs_on_way msg))
    (cl:cons ':ifSave (ifSave msg))
))
;//! \htmlinclude ToppRa_srv-response.msg.html

(cl:defclass <ToppRa_srv-response> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (vel
    :reader vel
    :initarg :vel
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (acc
    :reader acc
    :initarg :acc
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (t
    :reader t
    :initarg :t
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ToppRa_srv-response (<ToppRa_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToppRa_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToppRa_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_arm_toppra-srv:<ToppRa_srv-response> is deprecated: use rokae_arm_toppra-srv:ToppRa_srv-response instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <ToppRa_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_arm_toppra-srv:pos-val is deprecated.  Use rokae_arm_toppra-srv:pos instead.")
  (pos m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <ToppRa_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_arm_toppra-srv:vel-val is deprecated.  Use rokae_arm_toppra-srv:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <ToppRa_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_arm_toppra-srv:acc-val is deprecated.  Use rokae_arm_toppra-srv:acc instead.")
  (acc m))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <ToppRa_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_arm_toppra-srv:t-val is deprecated.  Use rokae_arm_toppra-srv:t instead.")
  (t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToppRa_srv-response>) ostream)
  "Serializes a message object of type '<ToppRa_srv-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'pos))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vel))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'acc))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 't))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToppRa_srv-response>) istream)
  "Deserializes a message object of type '<ToppRa_srv-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'vel) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'vel)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'acc) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'acc)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 't) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 't)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToppRa_srv-response>)))
  "Returns string type for a service object of type '<ToppRa_srv-response>"
  "rokae_arm_toppra/ToppRa_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToppRa_srv-response)))
  "Returns string type for a service object of type 'ToppRa_srv-response"
  "rokae_arm_toppra/ToppRa_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToppRa_srv-response>)))
  "Returns md5sum for a message object of type '<ToppRa_srv-response>"
  "7cd293dc242907880f3bae1412ae9b57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToppRa_srv-response)))
  "Returns md5sum for a message object of type 'ToppRa_srv-response"
  "7cd293dc242907880f3bae1412ae9b57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToppRa_srv-response>)))
  "Returns full string definition for message of type '<ToppRa_srv-response>"
  (cl:format cl:nil "float64[] pos~%float64[] vel~%float64[] acc~%float64[] t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToppRa_srv-response)))
  "Returns full string definition for message of type 'ToppRa_srv-response"
  (cl:format cl:nil "float64[] pos~%float64[] vel~%float64[] acc~%float64[] t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToppRa_srv-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'vel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'acc) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 't) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToppRa_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ToppRa_srv-response
    (cl:cons ':pos (pos msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':t (t msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ToppRa_srv)))
  'ToppRa_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ToppRa_srv)))
  'ToppRa_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToppRa_srv)))
  "Returns string type for a service object of type '<ToppRa_srv>"
  "rokae_arm_toppra/ToppRa_srv")