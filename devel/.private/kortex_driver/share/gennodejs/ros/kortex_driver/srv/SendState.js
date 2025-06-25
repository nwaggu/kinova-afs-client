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

class SendStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.indicies = null;
    }
    else {
      if (initObj.hasOwnProperty('indicies')) {
        this.indicies = initObj.indicies
      }
      else {
        this.indicies = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendStateRequest
    // Serialize message field [indicies]
    bufferOffset = _arraySerializer.int8(obj.indicies, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendStateRequest
    let len;
    let data = new SendStateRequest(null);
    // Deserialize message field [indicies]
    data.indicies = _arrayDeserializer.int8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.indicies.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SendStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59995a16fd277aa8e4abbdca6b7b9199';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8[] indicies
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendStateRequest(null);
    if (msg.indicies !== undefined) {
      resolved.indicies = msg.indicies;
    }
    else {
      resolved.indicies = []
    }

    return resolved;
    }
};

class SendStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SendStateResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SendStateResponse
    let len;
    let data = new SendStateResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SendStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SendStateResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SendStateRequest,
  Response: SendStateResponse,
  md5sum() { return '2db8f4ad9fe7fd5f3afc52b1481c1ea1'; },
  datatype() { return 'kortex_driver/SendState'; }
};
