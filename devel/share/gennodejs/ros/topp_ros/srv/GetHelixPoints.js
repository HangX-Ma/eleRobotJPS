// Auto-generated. Do not edit!

// (in-package topp_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let trajectory_msgs = _finder('trajectory_msgs');

//-----------------------------------------------------------

class GetHelixPointsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.r = null;
      this.angleStep = null;
      this.x0 = null;
      this.y0 = null;
      this.z0 = null;
      this.zf = null;
      this.deltaZ = null;
    }
    else {
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0.0;
      }
      if (initObj.hasOwnProperty('angleStep')) {
        this.angleStep = initObj.angleStep
      }
      else {
        this.angleStep = 0.0;
      }
      if (initObj.hasOwnProperty('x0')) {
        this.x0 = initObj.x0
      }
      else {
        this.x0 = 0.0;
      }
      if (initObj.hasOwnProperty('y0')) {
        this.y0 = initObj.y0
      }
      else {
        this.y0 = 0.0;
      }
      if (initObj.hasOwnProperty('z0')) {
        this.z0 = initObj.z0
      }
      else {
        this.z0 = 0.0;
      }
      if (initObj.hasOwnProperty('zf')) {
        this.zf = initObj.zf
      }
      else {
        this.zf = 0.0;
      }
      if (initObj.hasOwnProperty('deltaZ')) {
        this.deltaZ = initObj.deltaZ
      }
      else {
        this.deltaZ = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetHelixPointsRequest
    // Serialize message field [r]
    bufferOffset = _serializer.float64(obj.r, buffer, bufferOffset);
    // Serialize message field [angleStep]
    bufferOffset = _serializer.float64(obj.angleStep, buffer, bufferOffset);
    // Serialize message field [x0]
    bufferOffset = _serializer.float64(obj.x0, buffer, bufferOffset);
    // Serialize message field [y0]
    bufferOffset = _serializer.float64(obj.y0, buffer, bufferOffset);
    // Serialize message field [z0]
    bufferOffset = _serializer.float64(obj.z0, buffer, bufferOffset);
    // Serialize message field [zf]
    bufferOffset = _serializer.float64(obj.zf, buffer, bufferOffset);
    // Serialize message field [deltaZ]
    bufferOffset = _serializer.float64(obj.deltaZ, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetHelixPointsRequest
    let len;
    let data = new GetHelixPointsRequest(null);
    // Deserialize message field [r]
    data.r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angleStep]
    data.angleStep = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x0]
    data.x0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y0]
    data.y0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z0]
    data.z0 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [zf]
    data.zf = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [deltaZ]
    data.deltaZ = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'topp_ros/GetHelixPointsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4f144c19586ad3058e993b212b9c8d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 r
    float64 angleStep
    float64 x0
    float64 y0
    float64 z0
    float64 zf
    float64 deltaZ
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetHelixPointsRequest(null);
    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0.0
    }

    if (msg.angleStep !== undefined) {
      resolved.angleStep = msg.angleStep;
    }
    else {
      resolved.angleStep = 0.0
    }

    if (msg.x0 !== undefined) {
      resolved.x0 = msg.x0;
    }
    else {
      resolved.x0 = 0.0
    }

    if (msg.y0 !== undefined) {
      resolved.y0 = msg.y0;
    }
    else {
      resolved.y0 = 0.0
    }

    if (msg.z0 !== undefined) {
      resolved.z0 = msg.z0;
    }
    else {
      resolved.z0 = 0.0
    }

    if (msg.zf !== undefined) {
      resolved.zf = msg.zf;
    }
    else {
      resolved.zf = 0.0
    }

    if (msg.deltaZ !== undefined) {
      resolved.deltaZ = msg.deltaZ;
    }
    else {
      resolved.deltaZ = 0.0
    }

    return resolved;
    }
};

class GetHelixPointsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.helix_points = null;
    }
    else {
      if (initObj.hasOwnProperty('helix_points')) {
        this.helix_points = initObj.helix_points
      }
      else {
        this.helix_points = new trajectory_msgs.msg.JointTrajectory();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetHelixPointsResponse
    // Serialize message field [helix_points]
    bufferOffset = trajectory_msgs.msg.JointTrajectory.serialize(obj.helix_points, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetHelixPointsResponse
    let len;
    let data = new GetHelixPointsResponse(null);
    // Deserialize message field [helix_points]
    data.helix_points = trajectory_msgs.msg.JointTrajectory.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.JointTrajectory.getMessageSize(object.helix_points);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'topp_ros/GetHelixPointsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7652abeefc2221336a2308c3bf30bca3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    trajectory_msgs/JointTrajectory helix_points
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectory
    Header header
    string[] joint_names
    JointTrajectoryPoint[] points
    
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
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectoryPoint
    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration time_from_start
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetHelixPointsResponse(null);
    if (msg.helix_points !== undefined) {
      resolved.helix_points = trajectory_msgs.msg.JointTrajectory.Resolve(msg.helix_points)
    }
    else {
      resolved.helix_points = new trajectory_msgs.msg.JointTrajectory()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetHelixPointsRequest,
  Response: GetHelixPointsResponse,
  md5sum() { return '822f91513c2ae98aef72e6642f42233f'; },
  datatype() { return 'topp_ros/GetHelixPoints'; }
};
