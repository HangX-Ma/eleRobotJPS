;; Auto-generated. Do not edit!


(when (boundp 'rokae_jps_navigation::Goto)
  (if (not (find-package "ROKAE_JPS_NAVIGATION"))
    (make-package "ROKAE_JPS_NAVIGATION"))
  (shadow 'Goto (find-package "ROKAE_JPS_NAVIGATION")))
(unless (find-package "ROKAE_JPS_NAVIGATION::GOTO")
  (make-package "ROKAE_JPS_NAVIGATION::GOTO"))
(unless (find-package "ROKAE_JPS_NAVIGATION::GOTOREQUEST")
  (make-package "ROKAE_JPS_NAVIGATION::GOTOREQUEST"))
(unless (find-package "ROKAE_JPS_NAVIGATION::GOTORESPONSE")
  (make-package "ROKAE_JPS_NAVIGATION::GOTORESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass rokae_jps_navigation::GotoRequest
  :super ros::object
  :slots (_goal_pose _ifback ))

(defmethod rokae_jps_navigation::GotoRequest
  (:init
   (&key
    ((:goal_pose __goal_pose) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Pose :init) r)) r))
    ((:ifback __ifback) nil)
    )
   (send-super :init)
   (setq _goal_pose __goal_pose)
   (setq _ifback __ifback)
   self)
  (:goal_pose
   (&rest __goal_pose)
   (if (keywordp (car __goal_pose))
       (send* _goal_pose __goal_pose)
     (progn
       (if __goal_pose (setq _goal_pose (car __goal_pose)))
       _goal_pose)))
  (:ifback
   (&optional __ifback)
   (if __ifback (setq _ifback __ifback)) _ifback)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose[] _goal_pose
    (apply #'+ (send-all _goal_pose :serialization-length)) 4
    ;; bool _ifback
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose[] _goal_pose
     (write-long (length _goal_pose) s)
     (dolist (elem _goal_pose)
       (send elem :serialize s)
       )
     ;; bool _ifback
       (if _ifback (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose[] _goal_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal_pose (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _goal_pose)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _ifback
     (setq _ifback (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_jps_navigation::GotoResponse
  :super ros::object
  :slots (_message _success _pos _vel _acc _t _back_pos _back_vel _back_acc _back_t ))

(defmethod rokae_jps_navigation::GotoResponse
  (:init
   (&key
    ((:message __message) "")
    ((:success __success) nil)
    ((:pos __pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:acc __acc) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:t __t) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:back_pos __back_pos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:back_vel __back_vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:back_acc __back_acc) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:back_t __back_t) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _success __success)
   (setq _pos __pos)
   (setq _vel __vel)
   (setq _acc __acc)
   (setq _t __t)
   (setq _back_pos __back_pos)
   (setq _back_vel __back_vel)
   (setq _back_acc __back_acc)
   (setq _back_t __back_t)
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:back_pos
   (&optional __back_pos)
   (if __back_pos (setq _back_pos __back_pos)) _back_pos)
  (:back_vel
   (&optional __back_vel)
   (if __back_vel (setq _back_vel __back_vel)) _back_vel)
  (:back_acc
   (&optional __back_acc)
   (if __back_acc (setq _back_acc __back_acc)) _back_acc)
  (:back_t
   (&optional __back_t)
   (if __back_t (setq _back_t __back_t)) _back_t)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; bool _success
    1
    ;; float64[] _pos
    (* 8    (length _pos)) 4
    ;; float64[] _vel
    (* 8    (length _vel)) 4
    ;; float64[] _acc
    (* 8    (length _acc)) 4
    ;; float64[] _t
    (* 8    (length _t)) 4
    ;; float64[] _back_pos
    (* 8    (length _back_pos)) 4
    ;; float64[] _back_vel
    (* 8    (length _back_vel)) 4
    ;; float64[] _back_acc
    (* 8    (length _back_acc)) 4
    ;; float64[] _back_t
    (* 8    (length _back_t)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; float64[] _pos
     (write-long (length _pos) s)
     (dotimes (i (length _pos))
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _vel
     (write-long (length _vel) s)
     (dotimes (i (length _vel))
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _acc
     (write-long (length _acc) s)
     (dotimes (i (length _acc))
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _t
     (write-long (length _t) s)
     (dotimes (i (length _t))
       (sys::poke (elt _t i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _back_pos
     (write-long (length _back_pos) s)
     (dotimes (i (length _back_pos))
       (sys::poke (elt _back_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _back_vel
     (write-long (length _back_vel) s)
     (dotimes (i (length _back_vel))
       (sys::poke (elt _back_vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _back_acc
     (write-long (length _back_acc) s)
     (dotimes (i (length _back_acc))
       (sys::poke (elt _back_acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _back_t
     (write-long (length _back_t) s)
     (dotimes (i (length _back_t))
       (sys::poke (elt _back_t i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64[] _pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _acc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _acc (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _t
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _t (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _t i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _back_pos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _back_pos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _back_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _back_vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _back_vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _back_vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _back_acc
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _back_acc (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _back_acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _back_t
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _back_t (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _back_t i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass rokae_jps_navigation::Goto
  :super ros::object
  :slots ())

(setf (get rokae_jps_navigation::Goto :md5sum-) "8981a10f92f52c0004c5f25fd13239d0")
(setf (get rokae_jps_navigation::Goto :datatype-) "rokae_jps_navigation/Goto")
(setf (get rokae_jps_navigation::Goto :request) rokae_jps_navigation::GotoRequest)
(setf (get rokae_jps_navigation::Goto :response) rokae_jps_navigation::GotoResponse)

(defmethod rokae_jps_navigation::GotoRequest
  (:response () (instance rokae_jps_navigation::GotoResponse :init)))

(setf (get rokae_jps_navigation::GotoRequest :md5sum-) "8981a10f92f52c0004c5f25fd13239d0")
(setf (get rokae_jps_navigation::GotoRequest :datatype-) "rokae_jps_navigation/GotoRequest")
(setf (get rokae_jps_navigation::GotoRequest :definition-)
      "geometry_msgs/Pose[] goal_pose
bool ifback

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
string message
bool success
float64[] pos
float64[] vel
float64[] acc
float64[] t
float64[] back_pos
float64[] back_vel
float64[] back_acc
float64[] back_t
")

(setf (get rokae_jps_navigation::GotoResponse :md5sum-) "8981a10f92f52c0004c5f25fd13239d0")
(setf (get rokae_jps_navigation::GotoResponse :datatype-) "rokae_jps_navigation/GotoResponse")
(setf (get rokae_jps_navigation::GotoResponse :definition-)
      "geometry_msgs/Pose[] goal_pose
bool ifback

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
string message
bool success
float64[] pos
float64[] vel
float64[] acc
float64[] t
float64[] back_pos
float64[] back_vel
float64[] back_acc
float64[] back_t
")



(provide :rokae_jps_navigation/Goto "8981a10f92f52c0004c5f25fd13239d0")


