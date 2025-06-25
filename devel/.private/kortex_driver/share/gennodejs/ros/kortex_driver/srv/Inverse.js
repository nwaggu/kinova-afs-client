// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class InverseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_position = null;
      this.y_position = null;
      this.z_position = null;
    }
    else {
      if (initObj.hasOwnProperty('x_position')) {
        this.x_position = initObj.x_position
      }
      else {
        this.x_position = 0.0;
      }
      if (initObj.hasOwnProperty('y_position')) {
        this.y_position = initObj.y_position
      }
      else {
        this.y_position = 0.0;
      }
      if (initObj.hasOwnProperty('z_position')) {
        this.z_position = initObj.z_position
      }
      else {
        this.z_position = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InverseRequest
    // Serialize message field [x_position]
    bufferOffset = _serializer.float64(obj.x_position, buffer, bufferOffset);
    // Serialize message field [y_position]
    bufferOffset = _serializer.float64(obj.y_position, buffer, bufferOffset);
    // Serialize message field [z_position]
    bufferOffset = _serializer.float64(obj.z_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InverseRequest
    let len;
    let data = new InverseRequest(null);
    // Deserialize message field [x_position]
    data.x_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_position]
    data.y_position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_position]
    data.z_position = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/InverseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a65f5a0cdef54b2e1dbb78f7873209a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x_position
    float64 y_position
    float64 z_position
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InverseRequest(null);
    if (msg.x_position !== undefined) {
      resolved.x_position = msg.x_position;
    }
    else {
      resolved.x_position = 0.0
    }

    if (msg.y_position !== undefined) {
      resolved.y_position = msg.y_position;
    }
    else {
      resolved.y_position = 0.0
    }

    if (msg.z_position !== undefined) {
      resolved.z_position = msg.z_position;
    }
    else {
      resolved.z_position = 0.0
    }

    return resolved;
    }
};

class InverseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joints = null;
    }
    else {
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InverseResponse
    // Serialize message field [joints]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.joints, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InverseResponse
    let len;
    let data = new InverseResponse(null);
    // Deserialize message field [joints]
    data.joints = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.JointState.getMessageSize(object.joints);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/InverseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13b568889983e6c4080c58d8e7c2c89c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    sensor_msgs/JointState joints
    
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
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
    const resolved = new InverseResponse(null);
    if (msg.joints !== undefined) {
      resolved.joints = sensor_msgs.msg.JointState.Resolve(msg.joints)
    }
    else {
      resolved.joints = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

module.exports = {
  Request: InverseRequest,
  Response: InverseResponse,
  md5sum() { return '1652b6a55b9608f7a5bbc015e95332ff'; },
  datatype() { return 'kortex_driver/Inverse'; }
};
