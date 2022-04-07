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
        this.joint0 = 0.0;
      }
      if (initObj.hasOwnProperty('joint1')) {
        this.joint1 = initObj.joint1
      }
      else {
        this.joint1 = 0.0;
      }
      if (initObj.hasOwnProperty('joint2')) {
        this.joint2 = initObj.joint2
      }
      else {
        this.joint2 = 0.0;
      }
      if (initObj.hasOwnProperty('joint3')) {
        this.joint3 = initObj.joint3
      }
      else {
        this.joint3 = 0.0;
      }
      if (initObj.hasOwnProperty('joint4')) {
        this.joint4 = initObj.joint4
      }
      else {
        this.joint4 = 0.0;
      }
      if (initObj.hasOwnProperty('joint5')) {
        this.joint5 = initObj.joint5
      }
      else {
        this.joint5 = 0.0;
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
    bufferOffset = _serializer.float64(obj.joint0, buffer, bufferOffset);
    // Serialize message field [joint1]
    bufferOffset = _serializer.float64(obj.joint1, buffer, bufferOffset);
    // Serialize message field [joint2]
    bufferOffset = _serializer.float64(obj.joint2, buffer, bufferOffset);
    // Serialize message field [joint3]
    bufferOffset = _serializer.float64(obj.joint3, buffer, bufferOffset);
    // Serialize message field [joint4]
    bufferOffset = _serializer.float64(obj.joint4, buffer, bufferOffset);
    // Serialize message field [joint5]
    bufferOffset = _serializer.float64(obj.joint5, buffer, bufferOffset);
    // Serialize message field [ifVerbose]
    bufferOffset = _serializer.bool(obj.ifVerbose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint2poseRequest
    let len;
    let data = new joint2poseRequest(null);
    // Deserialize message field [joint0]
    data.joint0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint1]
    data.joint1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint2]
    data.joint2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint3]
    data.joint3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint4]
    data.joint4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint5]
    data.joint5 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ifVerbose]
    data.ifVerbose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/joint2poseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9ed341ddbb4f864aa1eec399727b045c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 joint0
    float64 joint1
    float64 joint2
    float64 joint3
    float64 joint4
    float64 joint5
    bool ifVerbose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joint2poseRequest(null);
    if (msg.joint0 !== undefined) {
      resolved.joint0 = msg.joint0;
    }
    else {
      resolved.joint0 = 0.0
    }

    if (msg.joint1 !== undefined) {
      resolved.joint1 = msg.joint1;
    }
    else {
      resolved.joint1 = 0.0
    }

    if (msg.joint2 !== undefined) {
      resolved.joint2 = msg.joint2;
    }
    else {
      resolved.joint2 = 0.0
    }

    if (msg.joint3 !== undefined) {
      resolved.joint3 = msg.joint3;
    }
    else {
      resolved.joint3 = 0.0
    }

    if (msg.joint4 !== undefined) {
      resolved.joint4 = msg.joint4;
    }
    else {
      resolved.joint4 = 0.0
    }

    if (msg.joint5 !== undefined) {
      resolved.joint5 = msg.joint5;
    }
    else {
      resolved.joint5 = 0.0
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
      this.re_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('re_pose')) {
        this.re_pose = initObj.re_pose
      }
      else {
        this.re_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joint2poseResponse
    // Serialize message field [re_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.re_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joint2poseResponse
    let len;
    let data = new joint2poseResponse(null);
    // Deserialize message field [re_pose]
    data.re_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/joint2poseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '222653e911ec2723bf153c3e2c46d638';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose re_pose
    
    
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
    const resolved = new joint2poseResponse(null);
    if (msg.re_pose !== undefined) {
      resolved.re_pose = geometry_msgs.msg.Pose.Resolve(msg.re_pose)
    }
    else {
      resolved.re_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: joint2poseRequest,
  Response: joint2poseResponse,
  md5sum() { return '8e3735eb3bfe1ca9ba91861cd4e5d3f8'; },
  datatype() { return 'rokae_jps_navigation/joint2pose'; }
};
