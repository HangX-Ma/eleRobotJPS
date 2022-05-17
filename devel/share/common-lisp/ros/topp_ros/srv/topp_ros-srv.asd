
(cl:in-package :asdf)

(defsystem "topp_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "GenerateTrajectory" :depends-on ("_package_GenerateTrajectory"))
    (:file "_package_GenerateTrajectory" :depends-on ("_package"))
    (:file "GetHelixPoints" :depends-on ("_package_GetHelixPoints"))
    (:file "_package_GetHelixPoints" :depends-on ("_package"))
  ))