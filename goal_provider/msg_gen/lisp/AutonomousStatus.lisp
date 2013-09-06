; Auto-generated. Do not edit!


(cl:in-package goal_provider-msg)


;//! \htmlinclude AutonomousStatus.msg.html

(cl:defclass <AutonomousStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (text
    :reader text
    :initarg :text
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String)))
)

(cl:defclass AutonomousStatus (<AutonomousStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AutonomousStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AutonomousStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name goal_provider-msg:<AutonomousStatus> is deprecated: use goal_provider-msg:AutonomousStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AutonomousStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader goal_provider-msg:header-val is deprecated.  Use goal_provider-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <AutonomousStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader goal_provider-msg:goal-val is deprecated.  Use goal_provider-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <AutonomousStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader goal_provider-msg:status-val is deprecated.  Use goal_provider-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <AutonomousStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader goal_provider-msg:text-val is deprecated.  Use goal_provider-msg:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AutonomousStatus>)))
    "Constants for message type '<AutonomousStatus>"
  '((:GOAL_ACTIVE . 1)
    (:GOAL_WAITING . 2)
    (:GOAL_REJECTED . 3)
    (:GOAL_ABORTED . 4)
    (:GOAL_REACHED . 5)
    (:LOST . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AutonomousStatus)))
    "Constants for message type 'AutonomousStatus"
  '((:GOAL_ACTIVE . 1)
    (:GOAL_WAITING . 2)
    (:GOAL_REJECTED . 3)
    (:GOAL_ABORTED . 4)
    (:GOAL_REACHED . 5)
    (:LOST . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AutonomousStatus>) ostream)
  "Serializes a message object of type '<AutonomousStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'text) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AutonomousStatus>) istream)
  "Deserializes a message object of type '<AutonomousStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'text) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AutonomousStatus>)))
  "Returns string type for a message object of type '<AutonomousStatus>"
  "goal_provider/AutonomousStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AutonomousStatus)))
  "Returns string type for a message object of type 'AutonomousStatus"
  "goal_provider/AutonomousStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AutonomousStatus>)))
  "Returns md5sum for a message object of type '<AutonomousStatus>"
  "94b6bb992d032536763367c493228167")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AutonomousStatus)))
  "Returns md5sum for a message object of type 'AutonomousStatus"
  "94b6bb992d032536763367c493228167")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AutonomousStatus>)))
  "Returns full string definition for message of type '<AutonomousStatus>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point goal            #The goal we are heading to~%~%uint8 status~%  uint8 GOAL_ACTIVE    = 1            #We are actively heading to a goal~%  uint8 GOAL_WAITING   = 2            #We are waiting for a goal to be accepted by move_base~%  uint8 GOAL_REJECTED  = 3            #Movebase rejected our goal~%  uint8 GOAL_ABORTED   = 4            #Movebase aborted our goal~%  uint8 GOAL_REACHED   = 5            #We have reached the goal and can start planning for the next one~%  uint8 LOST           = 6            #We are lost and don't know what is happening anymore. This is terminal~%~%std_msgs/String text                         #For extra information~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AutonomousStatus)))
  "Returns full string definition for message of type 'AutonomousStatus"
  (cl:format cl:nil "Header header~%geometry_msgs/Point goal            #The goal we are heading to~%~%uint8 status~%  uint8 GOAL_ACTIVE    = 1            #We are actively heading to a goal~%  uint8 GOAL_WAITING   = 2            #We are waiting for a goal to be accepted by move_base~%  uint8 GOAL_REJECTED  = 3            #Movebase rejected our goal~%  uint8 GOAL_ABORTED   = 4            #Movebase aborted our goal~%  uint8 GOAL_REACHED   = 5            #We have reached the goal and can start planning for the next one~%  uint8 LOST           = 6            #We are lost and don't know what is happening anymore. This is terminal~%~%std_msgs/String text                         #For extra information~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AutonomousStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AutonomousStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'AutonomousStatus
    (cl:cons ':header (header msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':status (status msg))
    (cl:cons ':text (text msg))
))
