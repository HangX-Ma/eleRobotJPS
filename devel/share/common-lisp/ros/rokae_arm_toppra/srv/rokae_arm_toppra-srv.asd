
(cl:in-package :asdf)

(defsystem "rokae_arm_toppra-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ToppRa_srv" :depends-on ("_package_ToppRa_srv"))
    (:file "_package_ToppRa_srv" :depends-on ("_package"))
  ))