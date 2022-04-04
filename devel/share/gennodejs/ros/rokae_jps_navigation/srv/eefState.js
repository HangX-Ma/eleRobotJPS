// Auto-generated. Do not edit!

// (in-package rokae_jps_navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class eefStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ifVerbose = null;
    }
    else {
      if (initObj.hasOwnProperty('ifVerbose')) {
        this.ifVerbose = initObj.ifVerbose
      }
      else {
        this.ifVerbose = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type eefStateRequest
    // Serialize message field [ifVerbose]
    bufferOffset = _serializer.bool(obj.ifVerbose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type eefStateRequest
    let len;
    let data = new eefStateRequest(null);
    // Deserialize message field [ifVerbose]
    data.ifVerbose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/eefStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67f516a089e3bfc934eaf4bae00ce98e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ifVerbose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new eefStateRequest(null);
    if (msg.ifVerbose !== undefined) {
      resolved.ifVerbose = msg.ifVerbose;
    }
    else {
      resolved.ifVerbose = false
    }

    return resolved;
    }
};

class eefStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.eef_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('eef_pose')) {
        this.eef_pose = initObj.eef_pose
      }
      else {
        this.eef_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type eefStateResponse
    // Serialize message field [eef_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.eef_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type eefStateResponse
    let len;
    let data = new eefStateResponse(null);
    // Deserialize message field [eef_pose]
    data.eef_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/eefStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e78f5a947f0283d2ebd6c70bccbefe0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose eef_pose
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new eefStateResponse(null);
    if (msg.eef_pose !== undefined) {
      resolved.eef_pose = geometry_msgs.msg.Pose.Resolve(msg.eef_pose)
    }
    else {
      resolved.eef_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: eefStateRequest,
  Response: eefStateResponse,
  md5sum() { return 'dc45c501754cc9c6c2ec61a0d5b19682'; },
  datatype() { return 'rokae_jps_navigation/eefState'; }
};
