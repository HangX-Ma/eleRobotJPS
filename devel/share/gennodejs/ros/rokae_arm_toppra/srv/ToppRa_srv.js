// Auto-generated. Do not edit!

// (in-package rokae_arm_toppra.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ToppRa_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_configs_on_way = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_configs_on_way')) {
        this.joint_configs_on_way = initObj.joint_configs_on_way
      }
      else {
        this.joint_configs_on_way = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToppRa_srvRequest
    // Serialize message field [joint_configs_on_way]
    bufferOffset = _arraySerializer.float64(obj.joint_configs_on_way, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToppRa_srvRequest
    let len;
    let data = new ToppRa_srvRequest(null);
    // Deserialize message field [joint_configs_on_way]
    data.joint_configs_on_way = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_configs_on_way.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_arm_toppra/ToppRa_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '733832a93b7ef333f057ff38c1f9011e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] joint_configs_on_way 
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ToppRa_srvRequest(null);
    if (msg.joint_configs_on_way !== undefined) {
      resolved.joint_configs_on_way = msg.joint_configs_on_way;
    }
    else {
      resolved.joint_configs_on_way = []
    }

    return resolved;
    }
};

class ToppRa_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos = null;
      this.vel = null;
      this.acc = null;
      this.t = null;
    }
    else {
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = [];
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = [];
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = [];
      }
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToppRa_srvResponse
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, null);
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, null);
    // Serialize message field [t]
    bufferOffset = _arraySerializer.float64(obj.t, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToppRa_srvResponse
    let len;
    let data = new ToppRa_srvResponse(null);
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [t]
    data.t = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.pos.length;
    length += 8 * object.vel.length;
    length += 8 * object.acc.length;
    length += 8 * object.t.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_arm_toppra/ToppRa_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '526eca1870e2912cc295f3206d28830b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] pos
    float64[] vel
    float64[] acc
    float64[] t
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ToppRa_srvResponse(null);
    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = []
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = []
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = []
    }

    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ToppRa_srvRequest,
  Response: ToppRa_srvResponse,
  md5sum() { return '7a9e423704160b7b83b4437739d703ae'; },
  datatype() { return 'rokae_arm_toppra/ToppRa_srv'; }
};
