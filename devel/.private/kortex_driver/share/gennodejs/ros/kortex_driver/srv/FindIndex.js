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


//-----------------------------------------------------------

class FindIndexRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FindIndexRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FindIndexRequest
    let len;
    let data = new FindIndexRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/FindIndexRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FindIndexRequest(null);
    return resolved;
    }
};

class FindIndexResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.grip = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = 0;
      }
      if (initObj.hasOwnProperty('grip')) {
        this.grip = initObj.grip
      }
      else {
        this.grip = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FindIndexResponse
    // Serialize message field [index]
    bufferOffset = _serializer.int8(obj.index, buffer, bufferOffset);
    // Serialize message field [grip]
    bufferOffset = _serializer.int8(obj.grip, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FindIndexResponse
    let len;
    let data = new FindIndexResponse(null);
    // Deserialize message field [index]
    data.index = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [grip]
    data.grip = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/FindIndexResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc335e982131613152f901a4d2b0914b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 index
    int8 grip
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FindIndexResponse(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = 0
    }

    if (msg.grip !== undefined) {
      resolved.grip = msg.grip;
    }
    else {
      resolved.grip = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: FindIndexRequest,
  Response: FindIndexResponse,
  md5sum() { return 'dc335e982131613152f901a4d2b0914b'; },
  datatype() { return 'kortex_driver/FindIndex'; }
};
