;; Auto-generated. Do not edit!


(when (boundp 'rokae_jps_navigation::CheckCollision)
  (if (not (find-package "ROKAE_JPS_NAVIGATION"))
    (make-package "ROKAE_JPS_NAVIGATION"))
  (shadow 'CheckCollision (find-package "ROKAE_JPS_NAVIGATION")))
(unless (find-package "ROKAE_JPS_NAVIGATION::CHECKCOLLISION")
  (make-package "ROKAE_JPS_NAVIGATION::CHECKCOLLISION"))
(unless (find-package "ROKAE_JPS_NAVIGATION::CHECKCOLLISIONREQUEST")
  (make-package "ROKAE_JPS_NAVIGATION::CHECKCOLLISIONREQUEST"))
(unless (find-package "ROKAE_JPS_NAVIGATION::CHECKCOLLISIONRESPONSE")
  (make-package "ROKAE_JPS_NAVIGATION::CHECKCOLLISIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass rokae_jps_navigation::CheckCollisionRequest
  :super ros::object
  :slots (_path_pose _prev_joints _ifVerbose ))

(defmethod rokae_jps_navigation::CheckCollisionRequest
  (:init
   (&key
    ((:path_pose __path_pose) (instance geometry_msgs::Pose :init))
    ((:prev_joints __prev_joints) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ifVerbose __ifVerbose) nil)
    )
   (send-super :init)
   (setq _path_pose __path_pose)
   (setq _prev_joints __prev_joints)
   (setq _ifVerbose __ifVerbose)
   self)
  (:path_pose
   (&rest __path_pose)
   (if (keywordp (car __path_pose))
       (send* _path_pose __path_pose)
     (progn
       (if __path_pose (setq _path_pose (car __path_pose)))
       _path_pose)))
  (:prev_joints
   (&optional __prev_joints)
   (if __prev_joints (setq _prev_joints __prev_joints)) _prev_joints)
  (:ifVerbose
   (&optional __ifVerbose)
   (if __ifVerbose (setq _ifVerbose __ifVerbose)) _ifVerbose)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _path_pose
    (send _path_pose :serialization-length)
    ;; float32[] _prev_joints
    (* 4    (length _prev_joints)) 4
    ;; bool _ifVerbose
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _path_pose
       (send _path_pose :serialize s)
     ;; float32[] _prev_joints
     (write-long (length _prev_joints) s)
     (dotimes (i (length _prev_joints))
       (sys::poke (elt _prev_joints i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _ifVerbose
       (if _ifVerbose (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _path_pose
     (send _path_pose :deserialize buf ptr-) (incf ptr- (send _path_pose :serialization-length))
   ;; float32[] _prev_joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _prev_joints (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _prev_joints i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; bool _ifVerbose
     (setq _ifVerbose (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rokae_jps_navigation::CheckCollisionResponse
  :super ros::object
  :slots (_isCollide _getSolution _curr_joints ))

(defmethod rokae_jps_navigation::CheckCollisionResponse
  (:init
   (&key
    ((:isCollide __isCollide) nil)
    ((:getSolution __getSolution) nil)
    ((:curr_joints __curr_joints) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _isCollide __isCollide)
   (setq _getSolution __getSolution)
   (setq _curr_joints __curr_joints)
   self)
  (:isCollide
   (&optional __isCollide)
   (if __isCollide (setq _isCollide __isCollide)) _isCollide)
  (:getSolution
   (&optional __getSolution)
   (if __getSolution (setq _getSolution __getSolution)) _getSolution)
  (:curr_joints
   (&optional __curr_joints)
   (if __curr_joints (setq _curr_joints __curr_joints)) _curr_joints)
  (:serialization-length
   ()
   (+
    ;; bool _isCollide
    1
    ;; bool _getSolution
    1
    ;; float32[] _curr_joints
    (* 4    (length _curr_joints)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isCollide
       (if _isCollide (write-byte -1 s) (write-byte 0 s))
     ;; bool _getSolution
       (if _getSolution (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _curr_joints
     (write-long (length _curr_joints) s)
     (dotimes (i (length _curr_joints))
       (sys::poke (elt _curr_joints i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isCollide
     (setq _isCollide (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _getSolution
     (setq _getSolution (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _curr_joints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _curr_joints (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _curr_joints i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass rokae_jps_navigation::CheckCollision
  :super ros::object
  :slots ())

(setf (get rokae_jps_navigation::CheckCollision :md5sum-) "9553736970fcba3d957b6a5220bb6872")
(setf (get rokae_jps_navigation::CheckCollision :datatype-) "rokae_jps_navigation/CheckCollision")
(setf (get rokae_jps_navigation::CheckCollision :request) rokae_jps_navigation::CheckCollisionRequest)
(setf (get rokae_jps_navigation::CheckCollision :response) rokae_jps_navigation::CheckCollisionResponse)

(defmethod rokae_jps_navigation::CheckCollisionRequest
  (:response () (instance rokae_jps_navigation::CheckCollisionResponse :init)))

(setf (get rokae_jps_navigation::CheckCollisionRequest :md5sum-) "9553736970fcba3d957b6a5220bb6872")
(setf (get rokae_jps_navigation::CheckCollisionRequest :datatype-) "rokae_jps_navigation/CheckCollisionRequest")
(setf (get rokae_jps_navigation::CheckCollisionRequest :definition-)
      "geometry_msgs/Pose path_pose
float32[] prev_joints
bool ifVerbose

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
bool isCollide
bool getSolution
float32[] curr_joints
")

(setf (get rokae_jps_navigation::CheckCollisionResponse :md5sum-) "9553736970fcba3d957b6a5220bb6872")
(setf (get rokae_jps_navigation::CheckCollisionResponse :datatype-) "rokae_jps_navigation/CheckCollisionResponse")
(setf (get rokae_jps_navigation::CheckCollisionResponse :definition-)
      "geometry_msgs/Pose path_pose
float32[] prev_joints
bool ifVerbose

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
bool isCollide
bool getSolution
float32[] curr_joints
")



(provide :rokae_jps_navigation/CheckCollision "9553736970fcba3d957b6a5220bb6872")


