// Auto-generated. Do not edit!

// (in-package dm_hw.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MotorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.names = null;
      this.position = null;
      this.velocity = null;
      this.effort = null;
      this.cmd_position = null;
      this.cmd_velocity = null;
      this.cmd_effort = null;
      this.kp = null;
      this.kd = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = [];
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = [];
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = [];
      }
      if (initObj.hasOwnProperty('cmd_position')) {
        this.cmd_position = initObj.cmd_position
      }
      else {
        this.cmd_position = [];
      }
      if (initObj.hasOwnProperty('cmd_velocity')) {
        this.cmd_velocity = initObj.cmd_velocity
      }
      else {
        this.cmd_velocity = [];
      }
      if (initObj.hasOwnProperty('cmd_effort')) {
        this.cmd_effort = initObj.cmd_effort
      }
      else {
        this.cmd_effort = [];
      }
      if (initObj.hasOwnProperty('kp')) {
        this.kp = initObj.kp
      }
      else {
        this.kp = [];
      }
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    // Serialize message field [position]
    bufferOffset = _arraySerializer.float64(obj.position, buffer, bufferOffset, null);
    // Serialize message field [velocity]
    bufferOffset = _arraySerializer.float64(obj.velocity, buffer, bufferOffset, null);
    // Serialize message field [effort]
    bufferOffset = _arraySerializer.float64(obj.effort, buffer, bufferOffset, null);
    // Serialize message field [cmd_position]
    bufferOffset = _arraySerializer.float64(obj.cmd_position, buffer, bufferOffset, null);
    // Serialize message field [cmd_velocity]
    bufferOffset = _arraySerializer.float64(obj.cmd_velocity, buffer, bufferOffset, null);
    // Serialize message field [cmd_effort]
    bufferOffset = _arraySerializer.float64(obj.cmd_effort, buffer, bufferOffset, null);
    // Serialize message field [kp]
    bufferOffset = _arraySerializer.float64(obj.kp, buffer, bufferOffset, null);
    // Serialize message field [kd]
    bufferOffset = _arraySerializer.float64(obj.kd, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorState
    let len;
    let data = new MotorState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [position]
    data.position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocity]
    data.velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort]
    data.effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmd_position]
    data.cmd_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmd_velocity]
    data.cmd_velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cmd_effort]
    data.cmd_effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [kp]
    data.kp = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [kd]
    data.kd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 8 * object.position.length;
    length += 8 * object.velocity.length;
    length += 8 * object.effort.length;
    length += 8 * object.cmd_position.length;
    length += 8 * object.cmd_velocity.length;
    length += 8 * object.cmd_effort.length;
    length += 8 * object.kp.length;
    length += 8 * object.kd.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dm_hw/MotorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f108578f1cff586bb9978189901903d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = []
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = []
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = []
    }

    if (msg.cmd_position !== undefined) {
      resolved.cmd_position = msg.cmd_position;
    }
    else {
      resolved.cmd_position = []
    }

    if (msg.cmd_velocity !== undefined) {
      resolved.cmd_velocity = msg.cmd_velocity;
    }
    else {
      resolved.cmd_velocity = []
    }

    if (msg.cmd_effort !== undefined) {
      resolved.cmd_effort = msg.cmd_effort;
    }
    else {
      resolved.cmd_effort = []
    }

    if (msg.kp !== undefined) {
      resolved.kp = msg.kp;
    }
    else {
      resolved.kp = []
    }

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = []
    }

    return resolved;
    }
};

module.exports = MotorState;
