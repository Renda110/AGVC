; Auto-generated. Do not edit!


(cl:in-package robot_comms-srv)


;//! \htmlinclude ImageFilter-request.msg.html

(cl:defclass <ImageFilter-request> (roslisp-msg-protocol:ros-message)
  ((filter_id
    :reader filter_id
    :initarg :filter_id
    :type cl:integer
    :initform 0))
)

(cl:defclass ImageFilter-request (<ImageFilter-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageFilter-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageFilter-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comms-srv:<ImageFilter-request> is deprecated: use robot_comms-srv:ImageFilter-request instead.")))

(cl:ensure-generic-function 'filter_id-val :lambda-list '(m))
(cl:defmethod filter_id-val ((m <ImageFilter-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comms-srv:filter_id-val is deprecated.  Use robot_comms-srv:filter_id instead.")
  (filter_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageFilter-request>) ostream)
  "Serializes a message object of type '<ImageFilter-request>"
  (cl:let* ((signed (cl:slot-value msg 'filter_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageFilter-request>) istream)
  "Deserializes a message object of type '<ImageFilter-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filter_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageFilter-request>)))
  "Returns string type for a service object of type '<ImageFilter-request>"
  "robot_comms/ImageFilterRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageFilter-request)))
  "Returns string type for a service object of type 'ImageFilter-request"
  "robot_comms/ImageFilterRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageFilter-request>)))
  "Returns md5sum for a message object of type '<ImageFilter-request>"
  "80825786ac8e53e4ca138e168c03ada7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageFilter-request)))
  "Returns md5sum for a message object of type 'ImageFilter-request"
  "80825786ac8e53e4ca138e168c03ada7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageFilter-request>)))
  "Returns full string definition for message of type '<ImageFilter-request>"
  (cl:format cl:nil "int64 filter_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageFilter-request)))
  "Returns full string definition for message of type 'ImageFilter-request"
  (cl:format cl:nil "int64 filter_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageFilter-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageFilter-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageFilter-request
    (cl:cons ':filter_id (filter_id msg))
))
;//! \htmlinclude ImageFilter-response.msg.html

(cl:defclass <ImageFilter-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass ImageFilter-response (<ImageFilter-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageFilter-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageFilter-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_comms-srv:<ImageFilter-response> is deprecated: use robot_comms-srv:ImageFilter-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ImageFilter-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_comms-srv:result-val is deprecated.  Use robot_comms-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageFilter-response>) ostream)
  "Serializes a message object of type '<ImageFilter-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageFilter-response>) istream)
  "Deserializes a message object of type '<ImageFilter-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageFilter-response>)))
  "Returns string type for a service object of type '<ImageFilter-response>"
  "robot_comms/ImageFilterResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageFilter-response)))
  "Returns string type for a service object of type 'ImageFilter-response"
  "robot_comms/ImageFilterResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageFilter-response>)))
  "Returns md5sum for a message object of type '<ImageFilter-response>"
  "80825786ac8e53e4ca138e168c03ada7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageFilter-response)))
  "Returns md5sum for a message object of type 'ImageFilter-response"
  "80825786ac8e53e4ca138e168c03ada7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageFilter-response>)))
  "Returns full string definition for message of type '<ImageFilter-response>"
  (cl:format cl:nil "int64 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageFilter-response)))
  "Returns full string definition for message of type 'ImageFilter-response"
  (cl:format cl:nil "int64 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageFilter-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageFilter-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageFilter-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ImageFilter)))
  'ImageFilter-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ImageFilter)))
  'ImageFilter-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageFilter)))
  "Returns string type for a service object of type '<ImageFilter>"
  "robot_comms/ImageFilter")