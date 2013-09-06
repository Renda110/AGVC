
(cl:in-package :asdf)

(defsystem "goal_provider-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AutonomousStatus" :depends-on ("_package_AutonomousStatus"))
    (:file "_package_AutonomousStatus" :depends-on ("_package"))
  ))