// Auto-generated. Do not edit!

// (in-package rokae_jps_navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class joint2poseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint0 = null;
      this.joint1 = null;
      this.joint2 = null;
      this.joint3 = null;
      this.joint4 = null;
      this.joint5 = null;
      this.ifVerbose = null;
    }
    else {
      if (initObj.hasOwnProperty('joint0')) {
        this.joint0 = initObj.joint0
      }
      else {
        this.joint0 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('joint1')) {
        this.joint1 = initObj.joint1
      }
      else {
        this.joint1 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('joint2')) {
        this.joint2 = initObj.joint2
      }
      else {
        this.joint2 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('joint3')) {
        this.joint3 = initObj.joint3
      }
      else {
        this.joint3 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('joint4')) {
        this.joint4 = initObj.joint4
      }
      else {
        this.joint4 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('joint5')) {
        this.joint5 = initObj.joint5
      }
      else {
        this.joint5 = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('ifVerbose')) {
        this.ifVerbose = initObj.ifVerbose
      }
      else {
        this.ifVerbose = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint2poseRequest
    // Serialize message field [joint0]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.joint0, buffer, bufferOffset);
    // Serialize message field [joint1]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.joint1, buffer, bufferOffset);
    // Serialize message field [joint2]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.joint2, buffer, bufferOffset);
    // Serialize message field [joint3]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.joint3, buffer, bufferOffset);
    // Serialize message field [joint4]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.joint4, buffer, bufferOffset);
    // Serialize message field [joint5]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.joint5, buffer, bufferOffset);
    // Serialize message field [ifVerbose]
    bufferOffset = _serializer.bool(obj.ifVerbose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint2poseRequest
    let len;
    let data = new joint2poseRequest(null);
    // Deserialize message field [joint0]
    data.joint0 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint1]
    data.joint1 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint2]
    data.joint2 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint3]
    data.joint3 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint4]
    data.joint4 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint5]
    data.joint5 = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [ifVerbose]
    data.ifVerbose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/joint2poseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5999e34ffaace7b38734bfad5de479e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32 joint0
    std_msgs/Float32 joint1
    std_msgs/Float32 joint2
    std_msgs/Float32 joint3
    std_msgs/Float32 joint4
    std_msgs/Float32 joint5
    bool ifVerbose
    
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint2poseRequest(null);
    if (msg.joint0 !== undefined) {
      resolved.joint0 = std_msgs.msg.Float32.Resolve(msg.joint0)
    }
    else {
      resolved.joint0 = new std_msgs.msg.Float32()
    }

    if (msg.joint1 !== undefined) {
      resolved.joint1 = std_msgs.msg.Float32.Resolve(msg.joint1)
    }
    else {
      resolved.joint1 = new std_msgs.msg.Float32()
    }

    if (msg.joint2 !== undefined) {
      resolved.joint2 = std_msgs.msg.Float32.Resolve(msg.joint2)
    }
    else {
      resolved.joint2 = new std_msgs.msg.Float32()
    }

    if (msg.joint3 !== undefined) {
      resolved.joint3 = std_msgs.msg.Float32.Resolve(msg.joint3)
    }
    else {
      resolved.joint3 = new std_msgs.msg.Float32()
    }

    if (msg.joint4 !== undefined) {
      resolved.joint4 = std_msgs.msg.Float32.Resolve(msg.joint4)
    }
    else {
      resolved.joint4 = new std_msgs.msg.Float32()
    }

    if (msg.joint5 !== undefined) {
      resolved.joint5 = std_msgs.msg.Float32.Resolve(msg.joint5)
    }
    else {
      resolved.joint5 = new std_msgs.msg.Float32()
    }

    if (msg.ifVerbose !== undefined) {
      resolved.ifVerbose = msg.ifVerbose;
    }
    else {
      resolved.ifVerbose = false
    }

    return resolved;
    }
};

class joint2poseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint2poseResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint2poseResponse
    let len;
    let data = new joint2poseResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/joint2poseResponse';
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
    const resolved = new joint2poseResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: joint2poseRequest,
  Response: joint2poseResponse,
  md5sum() { return '5999e34ffaace7b38734bfad5de479e9'; },
  datatype() { return 'rokae_jps_navigation/joint2pose'; }
};
