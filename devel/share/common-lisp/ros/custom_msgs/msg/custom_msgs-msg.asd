
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Pos" :depends-on ("_package_Pos"))
    (:file "_package_Pos" :depends-on ("_package"))
    (:file "navigationResult" :depends-on ("_package_navigationResult"))
    (:file "_package_navigationResult" :depends-on ("_package"))
    (:file "navigationFeedback" :depends-on ("_package_navigationFeedback"))
    (:file "_package_navigationFeedback" :depends-on ("_package"))
    (:file "navigationAction" :depends-on ("_package_navigationAction"))
    (:file "_package_navigationAction" :depends-on ("_package"))
    (:file "navigationActionGoal" :depends-on ("_package_navigationActionGoal"))
    (:file "_package_navigationActionGoal" :depends-on ("_package"))
    (:file "navigationGoal" :depends-on ("_package_navigationGoal"))
    (:file "_package_navigationGoal" :depends-on ("_package"))
    (:file "navigationActionFeedback" :depends-on ("_package_navigationActionFeedback"))
    (:file "_package_navigationActionFeedback" :depends-on ("_package"))
    (:file "navigationActionResult" :depends-on ("_package_navigationActionResult"))
    (:file "_package_navigationActionResult" :depends-on ("_package"))
  ))