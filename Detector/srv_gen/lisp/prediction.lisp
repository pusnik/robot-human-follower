; Auto-generated. Do not edit!


(cl:in-package Detector-srv)


;//! \htmlinclude prediction-request.msg.html

(cl:defclass <prediction-request> (roslisp-msg-protocol:ros-message)
  ((str
    :reader str
    :initarg :str
    :type cl:string
    :initform ""))
)

(cl:defclass prediction-request (<prediction-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <prediction-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'prediction-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Detector-srv:<prediction-request> is deprecated: use Detector-srv:prediction-request instead.")))

(cl:ensure-generic-function 'str-val :lambda-list '(m))
(cl:defmethod str-val ((m <prediction-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Detector-srv:str-val is deprecated.  Use Detector-srv:str instead.")
  (str m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <prediction-request>) ostream)
  "Serializes a message object of type '<prediction-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'str))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'str))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <prediction-request>) istream)
  "Deserializes a message object of type '<prediction-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'str) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'str) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<prediction-request>)))
  "Returns string type for a service object of type '<prediction-request>"
  "Detector/predictionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'prediction-request)))
  "Returns string type for a service object of type 'prediction-request"
  "Detector/predictionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<prediction-request>)))
  "Returns md5sum for a message object of type '<prediction-request>"
  "e6cc5d1c76477e8379a859ec77194d1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'prediction-request)))
  "Returns md5sum for a message object of type 'prediction-request"
  "e6cc5d1c76477e8379a859ec77194d1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<prediction-request>)))
  "Returns full string definition for message of type '<prediction-request>"
  (cl:format cl:nil "string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'prediction-request)))
  "Returns full string definition for message of type 'prediction-request"
  (cl:format cl:nil "string str~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <prediction-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'str))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <prediction-request>))
  "Converts a ROS message object to a list"
  (cl:list 'prediction-request
    (cl:cons ':str (str msg))
))
;//! \htmlinclude prediction-response.msg.html

(cl:defclass <prediction-response> (roslisp-msg-protocol:ros-message)
  ((prediction
    :reader prediction
    :initarg :prediction
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass prediction-response (<prediction-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <prediction-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'prediction-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Detector-srv:<prediction-response> is deprecated: use Detector-srv:prediction-response instead.")))

(cl:ensure-generic-function 'prediction-val :lambda-list '(m))
(cl:defmethod prediction-val ((m <prediction-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Detector-srv:prediction-val is deprecated.  Use Detector-srv:prediction instead.")
  (prediction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <prediction-response>) ostream)
  "Serializes a message object of type '<prediction-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'prediction) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <prediction-response>) istream)
  "Deserializes a message object of type '<prediction-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'prediction) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<prediction-response>)))
  "Returns string type for a service object of type '<prediction-response>"
  "Detector/predictionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'prediction-response)))
  "Returns string type for a service object of type 'prediction-response"
  "Detector/predictionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<prediction-response>)))
  "Returns md5sum for a message object of type '<prediction-response>"
  "e6cc5d1c76477e8379a859ec77194d1d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'prediction-response)))
  "Returns md5sum for a message object of type 'prediction-response"
  "e6cc5d1c76477e8379a859ec77194d1d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<prediction-response>)))
  "Returns full string definition for message of type '<prediction-response>"
  (cl:format cl:nil "geometry_msgs/Vector3 						prediction~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'prediction-response)))
  "Returns full string definition for message of type 'prediction-response"
  (cl:format cl:nil "geometry_msgs/Vector3 						prediction~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <prediction-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'prediction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <prediction-response>))
  "Converts a ROS message object to a list"
  (cl:list 'prediction-response
    (cl:cons ':prediction (prediction msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'prediction)))
  'prediction-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'prediction)))
  'prediction-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'prediction)))
  "Returns string type for a service object of type '<prediction>"
  "Detector/prediction")