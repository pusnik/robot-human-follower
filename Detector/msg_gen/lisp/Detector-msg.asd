
(cl:in-package :asdf)

(defsystem "Detector-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
               :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "Detection" :depends-on ("_package_Detection"))
    (:file "_package_Detection" :depends-on ("_package"))
    (:file "DetectionList" :depends-on ("_package_DetectionList"))
    (:file "_package_DetectionList" :depends-on ("_package"))
  ))