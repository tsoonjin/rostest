; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude Pos.msg.html

(cl:defclass <Pos> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Pos (<Pos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<Pos> is deprecated: use custom_msgs-msg:Pos instead.")))

(cl:ensure-generic-function 'pos_x-val :lambda-list '(m))
(cl:defmethod pos_x-val ((m <Pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:pos_x-val is deprecated.  Use custom_msgs-msg:pos_x instead.")
  (pos_x m))

(cl:ensure-generic-function 'pos_y-val :lambda-list '(m))
(cl:defmethod pos_y-val ((m <Pos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:pos_y-val is deprecated.  Use custom_msgs-msg:pos_y instead.")
  (pos_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pos>) ostream)
  "Serializes a message object of type '<Pos>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pos>) istream)
  "Deserializes a message object of type '<Pos>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pos>)))
  "Returns string type for a message object of type '<Pos>"
  "custom_msgs/Pos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pos)))
  "Returns string type for a message object of type 'Pos"
  "custom_msgs/Pos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pos>)))
  "Returns md5sum for a message object of type '<Pos>"
  "2178591bb5b4f72502724261422523b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pos)))
  "Returns md5sum for a message object of type 'Pos"
  "2178591bb5b4f72502724261422523b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pos>)))
  "Returns full string definition for message of type '<Pos>"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pos)))
  "Returns full string definition for message of type 'Pos"
  (cl:format cl:nil "float32 pos_x~%float32 pos_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pos>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pos>))
  "Converts a ROS message object to a list"
  (cl:list 'Pos
    (cl:cons ':pos_x (pos_x msg))
    (cl:cons ':pos_y (pos_y msg))
))
