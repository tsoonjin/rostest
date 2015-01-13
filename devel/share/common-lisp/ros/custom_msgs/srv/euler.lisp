; Auto-generated. Do not edit!


(cl:in-package custom_msgs-srv)


;//! \htmlinclude euler-request.msg.html

(cl:defclass <euler-request> (roslisp-msg-protocol:ros-message)
  ((pos_x
    :reader pos_x
    :initarg :pos_x
    :type cl:float
    :initform 0.0)
   (pos_y
    :reader pos_y
    :initarg :pos_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass euler-request (<euler-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <euler-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'euler-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<euler-request> is deprecated: use custom_msgs-srv:euler-request instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <euler-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:pos_x-val is deprecated.  Use custom_msgs-srv:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <euler-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:pos_y-val is deprecated.  Use custom_msgs-srv:pos_y instead.")
  (pos_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <euler-request>) ostream)
  "Serializes a message object of type '<euler-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <euler-request>) istream)
  "Deserializes a message object of type '<euler-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<euler-request>)))
  "Returns string type for a service object of type '<euler-request>"
  "custom_msgs/eulerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'euler-request)))
  "Returns string type for a service object of type 'euler-request"
  "custom_msgs/eulerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<euler-request>)))
  "Returns md5sum for a message object of type '<euler-request>"
  "0f0e6028382ae8d43564a2ad2bb6f998")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'euler-request)))
  "Returns md5sum for a message object of type 'euler-request"
  "0f0e6028382ae8d43564a2ad2bb6f998")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<euler-request>)))
  "Returns full string definition for message of type '<euler-request>"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'euler-request)))
  "Returns full string definition for message of type 'euler-request"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <euler-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <euler-request>))
  "Converts a ROS message object to a list"
  (cl:list 'euler-request
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
))
;//! \htmlinclude euler-response.msg.html

(cl:defclass <euler-response> (roslisp-msg-protocol:ros-message)
  ((pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass euler-response (<euler-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <euler-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'euler-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-srv:<euler-response> is deprecated: use custom_msgs-srv:euler-response instead.")))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <euler-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-srv:pos_z-val is deprecated.  Use custom_msgs-srv:pos_z instead.")
  (pos_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <euler-response>) ostream)
  "Serializes a message object of type '<euler-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pos_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <euler-response>) istream)
  "Deserializes a message object of type '<euler-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<euler-response>)))
  "Returns string type for a service object of type '<euler-response>"
  "custom_msgs/eulerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'euler-response)))
  "Returns string type for a service object of type 'euler-response"
  "custom_msgs/eulerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<euler-response>)))
  "Returns md5sum for a message object of type '<euler-response>"
  "0f0e6028382ae8d43564a2ad2bb6f998")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'euler-response)))
  "Returns md5sum for a message object of type 'euler-response"
  "0f0e6028382ae8d43564a2ad2bb6f998")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<euler-response>)))
  "Returns full string definition for message of type '<euler-response>"
  (cl:format cl:nil "float32 pos_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'euler-response)))
  "Returns full string definition for message of type 'euler-response"
  (cl:format cl:nil "float32 pos_z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <euler-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <euler-response>))
  "Converts a ROS message object to a list"
  (cl:list 'euler-response
    (cl:cons ':pos_z (pos_z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'euler)))
  'euler-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'euler)))
  'euler-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'euler)))
  "Returns string type for a service object of type '<euler>"
  "custom_msgs/euler")