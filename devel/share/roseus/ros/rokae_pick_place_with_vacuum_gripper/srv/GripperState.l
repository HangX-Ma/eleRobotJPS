;; Auto-generated. Do not edit!


(when (boundp 'rokae_pick_place_with_vacuum_gripper::GripperState)
  (if (not (find-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER"))
    (make-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER"))
  (shadow 'GripperState (find-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER")))
(unless (find-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER::GRIPPERSTATE")
  (make-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER::GRIPPERSTATE"))
(unless (find-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER::GRIPPERSTATEREQUEST")
  (make-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER::GRIPPERSTATEREQUEST"))
(unless (find-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER::GRIPPERSTATERESPONSE")
  (make-package "ROKAE_PICK_PLACE_WITH_VACUUM_GRIPPER::GRIPPERSTATERESPONSE"))

(in-package "ROS")





(defclass rokae_pick_place_with_vacuum_gripper::GripperStateRequest
  :super ros::object
  :slots (_switch_state ))

(defmethod rokae_pick_place_with_vacuum_gripper::GripperStateRequest
  (:init
   (&key
    ((:switch_state __switch_state) nil)
    )
   (send-super :init)
   (setq _switch_state __switch_state)
   self)
  (:switch_state
   (&optional __switch_state)
   (if __switch_state (setq _switch_state __switch_state)) _switch_state)
  (:serialization-length
   ()
   (+
    ;; bool _switch_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _switch_state
       (if _switch_state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _switch_state
     (setq _switch_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_pick_place_with_vacuum_gripper::GripperStateResponse
  :super ros::object
  :slots (_feedback ))

(defmethod rokae_pick_place_with_vacuum_gripper::GripperStateResponse
  (:init
   (&key
    ((:feedback __feedback) "")
    )
   (send-super :init)
   (setq _feedback (string __feedback))
   self)
  (:feedback
   (&optional __feedback)
   (if __feedback (setq _feedback __feedback)) _feedback)
  (:serialization-length
   ()
   (+
    ;; string _feedback
    4 (length _feedback)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _feedback
       (write-long (length _feedback) s) (princ _feedback s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _feedback
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _feedback (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rokae_pick_place_with_vacuum_gripper::GripperState
  :super ros::object
  :slots ())

(setf (get rokae_pick_place_with_vacuum_gripper::GripperState :md5sum-) "54728c51db3fb99fe3505e3f6f89c845")
(setf (get rokae_pick_place_with_vacuum_gripper::GripperState :datatype-) "rokae_pick_place_with_vacuum_gripper/GripperState")
(setf (get rokae_pick_place_with_vacuum_gripper::GripperState :request) rokae_pick_place_with_vacuum_gripper::GripperStateRequest)
(setf (get rokae_pick_place_with_vacuum_gripper::GripperState :response) rokae_pick_place_with_vacuum_gripper::GripperStateResponse)

(defmethod rokae_pick_place_with_vacuum_gripper::GripperStateRequest
  (:response () (instance rokae_pick_place_with_vacuum_gripper::GripperStateResponse :init)))

(setf (get rokae_pick_place_with_vacuum_gripper::GripperStateRequest :md5sum-) "54728c51db3fb99fe3505e3f6f89c845")
(setf (get rokae_pick_place_with_vacuum_gripper::GripperStateRequest :datatype-) "rokae_pick_place_with_vacuum_gripper/GripperStateRequest")
(setf (get rokae_pick_place_with_vacuum_gripper::GripperStateRequest :definition-)
      "# content of GripperState.srv

#request fields
bool switch_state
---
#response fields
string feedback

")

(setf (get rokae_pick_place_with_vacuum_gripper::GripperStateResponse :md5sum-) "54728c51db3fb99fe3505e3f6f89c845")
(setf (get rokae_pick_place_with_vacuum_gripper::GripperStateResponse :datatype-) "rokae_pick_place_with_vacuum_gripper/GripperStateResponse")
(setf (get rokae_pick_place_with_vacuum_gripper::GripperStateResponse :definition-)
      "# content of GripperState.srv

#request fields
bool switch_state
---
#response fields
string feedback

")



(provide :rokae_pick_place_with_vacuum_gripper/GripperState "54728c51db3fb99fe3505e3f6f89c845")


