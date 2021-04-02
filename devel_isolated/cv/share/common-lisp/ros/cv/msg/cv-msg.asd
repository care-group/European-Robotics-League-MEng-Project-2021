
(cl:in-package :asdf)

(defsystem "cv-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SemanticLabel" :depends-on ("_package_SemanticLabel"))
    (:file "_package_SemanticLabel" :depends-on ("_package"))
  ))