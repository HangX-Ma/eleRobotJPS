// Auto-generated. Do not edit!

// (in-package rokae_pick_place_with_vacuum_gripper.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GripperStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.switch_state = null;
    }
    else {
      if (initObj.hasOwnProperty('switch_state')) {
        this.switch_state = initObj.switch_state
      }
      else {
        this.switch_state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperStateRequest
    // Serialize message field [switch_state]
    bufferOffset = _serializer.bool(obj.switch_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperStateRequest
    let len;
    let data = new GripperStateRequest(null);
    // Deserialize message field [switch_state]
    data.switch_state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_pick_place_with_vacuum_gripper/GripperStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d03b6a3cc8309347b5a0df1aa787995';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # content of GripperState.srv
    
    #request fields
    bool switch_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperStateRequest(null);
    if (msg.switch_state !== undefined) {
      resolved.switch_state = msg.switch_state;
    }
    else {
      resolved.switch_state = false
    }

    return resolved;
    }
};

class GripperStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('feedback')) {
        this.feedback = initObj.feedback
      }
      else {
        this.feedback = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperStateResponse
    // Serialize message field [feedback]
    bufferOffset = _serializer.string(obj.feedback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperStateResponse
    let len;
    let data = new GripperStateResponse(null);
    // Deserialize message field [feedback]
    data.feedback = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.feedback.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_pick_place_with_vacuum_gripper/GripperStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c14cdf907e5c7c7fd47292add15660f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #response fields
    string feedback
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperStateResponse(null);
    if (msg.feedback !== undefined) {
      resolved.feedback = msg.feedback;
    }
    else {
      resolved.feedback = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GripperStateRequest,
  Response: GripperStateResponse,
  md5sum() { return '54728c51db3fb99fe3505e3f6f89c845'; },
  datatype() { return 'rokae_pick_place_with_vacuum_gripper/GripperState'; }
};
