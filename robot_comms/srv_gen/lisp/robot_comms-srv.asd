
(cl:in-package :asdf)

(defsystem "robot_comms-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ImageFilter" :depends-on ("_package_ImageFilter"))
    (:file "_package_ImageFilter" :depends-on ("_package"))
  ))