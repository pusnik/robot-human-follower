
(cl:in-package :asdf)

(defsystem "Detector-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "prediction" :depends-on ("_package_prediction"))
    (:file "_package_prediction" :depends-on ("_package"))
  ))