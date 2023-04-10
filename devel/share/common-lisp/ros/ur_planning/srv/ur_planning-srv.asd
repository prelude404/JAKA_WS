
(cl:in-package :asdf)

(defsystem "ur_planning-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "grasp_pose" :depends-on ("_package_grasp_pose"))
    (:file "_package_grasp_pose" :depends-on ("_package"))
  ))