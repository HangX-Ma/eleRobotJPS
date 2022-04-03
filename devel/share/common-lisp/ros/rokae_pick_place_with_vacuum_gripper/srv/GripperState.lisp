; Auto-generated. Do not edit!


(cl:in-package rokae_pick_place_with_vacuum_gripper-srv)


;//! \htmlinclude GripperState-request.msg.html

(cl:defclass <GripperState-request> (roslisp-msg-protocol:ros-message)
  ((switch_state
    :reader switch_state
    :initarg :switch_state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GripperState-request (<GripperState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_pick_place_with_vacuum_gripper-srv:<GripperState-request> is deprecated: use rokae_pick_place_with_vacuum_gripper-srv:GripperState-request instead.")))

(cl:ensure-generic-function 'switch_state-val :lambda-list '(m))
(cl:defmethod switch_state-val ((m <GripperState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_pick_place_with_vacuum_gripper-srv:switch_state-val is deprecated.  Use rokae_pick_place_with_vacuum_gripper-srv:switch_state instead.")
  (switch_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperState-request>) ostream)
  "Serializes a message object of type '<GripperState-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'switch_state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperState-request>) istream)
  "Deserializes a message object of type '<GripperState-request>"
    (cl:setf (cl:slot-value msg 'switch_state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperState-request>)))
  "Returns string type for a service object of type '<GripperState-request>"
  "rokae_pick_place_with_vacuum_gripper/GripperStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperState-request)))
  "Returns string type for a service object of type 'GripperState-request"
  "rokae_pick_place_with_vacuum_gripper/GripperStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperState-request>)))
  "Returns md5sum for a message object of type '<GripperState-request>"
  "54728c51db3fb99fe3505e3f6f89c845")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperState-request)))
  "Returns md5sum for a message object of type 'GripperState-request"
  "54728c51db3fb99fe3505e3f6f89c845")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperState-request>)))
  "Returns full string definition for message of type '<GripperState-request>"
  (cl:format cl:nil "# content of GripperState.srv~%~%#request fields~%bool switch_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperState-request)))
  "Returns full string definition for message of type 'GripperState-request"
  (cl:format cl:nil "# content of GripperState.srv~%~%#request fields~%bool switch_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperState-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperState-request
    (cl:cons ':switch_state (switch_state msg))
))
;//! \htmlinclude GripperState-response.msg.html

(cl:defclass <GripperState-response> (roslisp-msg-protocol:ros-message)
  ((feedback
    :reader feedback
    :initarg :feedback
    :type cl:string
    :initform ""))
)

(cl:defclass GripperState-response (<GripperState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rokae_pick_place_with_vacuum_gripper-srv:<GripperState-response> is deprecated: use rokae_pick_place_with_vacuum_gripper-srv:GripperState-response instead.")))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <GripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rokae_pick_place_with_vacuum_gripper-srv:feedback-val is deprecated.  Use rokae_pick_place_with_vacuum_gripper-srv:feedback instead.")
  (feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperState-response>) ostream)
  "Serializes a message object of type '<GripperState-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'feedback))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'feedback))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperState-response>) istream)
  "Deserializes a message object of type '<GripperState-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feedback) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'feedback) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperState-response>)))
  "Returns string type for a service object of type '<GripperState-response>"
  "rokae_pick_place_with_vacuum_gripper/GripperStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperState-response)))
  "Returns string type for a service object of type 'GripperState-response"
  "rokae_pick_place_with_vacuum_gripper/GripperStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperState-response>)))
  "Returns md5sum for a message object of type '<GripperState-response>"
  "54728c51db3fb99fe3505e3f6f89c845")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperState-response)))
  "Returns md5sum for a message object of type 'GripperState-response"
  "54728c51db3fb99fe3505e3f6f89c845")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperState-response>)))
  "Returns full string definition for message of type '<GripperState-response>"
  (cl:format cl:nil "#response fields~%string feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperState-response)))
  "Returns full string definition for message of type 'GripperState-response"
  (cl:format cl:nil "#response fields~%string feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperState-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperState-response
    (cl:cons ':feedback (feedback msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GripperState)))
  'GripperState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GripperState)))
  'GripperState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperState)))
  "Returns string type for a service object of type '<GripperState>"
  "rokae_pick_place_with_vacuum_gripper/GripperState")