;; Auto-generated. Do not edit!


(when (boundp 'dm_hw::MotorState)
  (if (not (find-package "DM_HW"))
    (make-package "DM_HW"))
  (shadow 'MotorState (find-package "DM_HW")))
(unless (find-package "DM_HW::MOTORSTATE")
  (make-package "DM_HW::MOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dm_hw::MotorState
  :super ros::object
  :slots (_header _names _position _velocity _effort _cmd_position _cmd_velocity _cmd_effort _kp _kd ))

(defmethod dm_hw::MotorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort __effort) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmd_position __cmd_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmd_velocity __cmd_velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmd_effort __cmd_effort) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:kp __kp) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:kd __kd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _names __names)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _effort __effort)
   (setq _cmd_position __cmd_position)
   (setq _cmd_velocity __cmd_velocity)
   (setq _cmd_effort __cmd_effort)
   (setq _kp __kp)
   (setq _kd __kd)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:cmd_position
   (&optional __cmd_position)
   (if __cmd_position (setq _cmd_position __cmd_position)) _cmd_position)
  (:cmd_velocity
   (&optional __cmd_velocity)
   (if __cmd_velocity (setq _cmd_velocity __cmd_velocity)) _cmd_velocity)
  (:cmd_effort
   (&optional __cmd_effort)
   (if __cmd_effort (setq _cmd_effort __cmd_effort)) _cmd_effort)
  (:kp
   (&optional __kp)
   (if __kp (setq _kp __kp)) _kp)
  (:kd
   (&optional __kd)
   (if __kd (setq _kd __kd)) _kd)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _effort
    (* 8    (length _effort)) 4
    ;; float64[] _cmd_position
    (* 8    (length _cmd_position)) 4
    ;; float64[] _cmd_velocity
    (* 8    (length _cmd_velocity)) 4
    ;; float64[] _cmd_effort
    (* 8    (length _cmd_effort)) 4
    ;; float64[] _kp
    (* 8    (length _kp)) 4
    ;; float64[] _kd
    (* 8    (length _kd)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort
     (write-long (length _effort) s)
     (dotimes (i (length _effort))
       (sys::poke (elt _effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmd_position
     (write-long (length _cmd_position) s)
     (dotimes (i (length _cmd_position))
       (sys::poke (elt _cmd_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmd_velocity
     (write-long (length _cmd_velocity) s)
     (dotimes (i (length _cmd_velocity))
       (sys::poke (elt _cmd_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmd_effort
     (write-long (length _cmd_effort) s)
     (dotimes (i (length _cmd_effort))
       (sys::poke (elt _cmd_effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _kp
     (write-long (length _kp) s)
     (dotimes (i (length _kp))
       (sys::poke (elt _kp i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _kd
     (write-long (length _kd) s)
     (dotimes (i (length _kd))
       (sys::poke (elt _kd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmd_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmd_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmd_effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _kp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kp (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kp i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _kd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _kd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _kd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get dm_hw::MotorState :md5sum-) "4f108578f1cff586bb9978189901903d")
(setf (get dm_hw::MotorState :datatype-) "dm_hw/MotorState")
(setf (get dm_hw::MotorState :definition-)
      "std_msgs/Header header

# Motor names (corresponds to indices in arrays below)
string[] names

# Current state values
float64[] position      # Current position of each motor
float64[] velocity      # Current velocity of each motor
float64[] effort        # Current effort/torque of each motor

# Command values
float64[] cmd_position  # Commanded position of each motor
float64[] cmd_velocity  # Commanded velocity of each motor
float64[] cmd_effort    # Commanded effort/torque of each motor

# Control parameters
float64[] kp            # Proportional gain for each motor
float64[] kd            # Derivative gain for each motor


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :dm_hw/MotorState "4f108578f1cff586bb9978189901903d")


