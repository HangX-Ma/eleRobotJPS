// Auto-generated. Do not edit!

// (in-package topp_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let trajectory_msgs = _finder('trajectory_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GenerateTrajectoryRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.waypoints = null;
      this.sampling_frequency = null;
      this.n_gridpoints = null;
      this.plot = null;
    }
    else {
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = new trajectory_msgs.msg.JointTrajectory();
      }
      if (initObj.hasOwnProperty('sampling_frequency')) {
        this.sampling_frequency = initObj.sampling_frequency
      }
      else {
        this.sampling_frequency = 0.0;
      }
      if (initObj.hasOwnProperty('n_gridpoints')) {
        this.n_gridpoints = initObj.n_gridpoints
      }
      else {
        this.n_gridpoints = 0;
      }
      if (initObj.hasOwnProperty('plot')) {
        this.plot = initObj.plot
      }
      else {
        this.plot = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GenerateTrajectoryRequest
    // Serialize message field [waypoints]
    bufferOffset = trajectory_msgs.msg.JointTrajectory.serialize(obj.waypoints, buffer, bufferOffset);
    // Serialize message field [sampling_frequency]
    bufferOffset = _serializer.float64(obj.sampling_frequency, buffer, bufferOffset);
    // Serialize message field [n_gridpoints]
    bufferOffset = _serializer.int32(obj.n_gridpoints, buffer, bufferOffset);
    // Serialize message field [plot]
    bufferOffset = _serializer.bool(obj.plot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GenerateTrajectoryRequest
    let len;
    let data = new GenerateTrajectoryRequest(null);
    // Deserialize message field [waypoints]
    data.waypoints = trajectory_msgs.msg.JointTrajectory.deserialize(buffer, bufferOffset);
    // Deserialize message field [sampling_frequency]
    data.sampling_frequency = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [n_gridpoints]
    data.n_gridpoints = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [plot]
    data.plot = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.JointTrajectory.getMessageSize(object.waypoints);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'topp_ros/GenerateTrajectoryRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a7db67aa372818c32ae772033caea39';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    trajectory_msgs/JointTrajectory     waypoints
    float64                             sampling_frequency
    int32                               n_gridpoints
    bool                                plot
    
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
    const resolved = new GenerateTrajectoryRequest(null);
    if (msg.waypoints !== undefined) {
      resolved.waypoints = trajectory_msgs.msg.JointTrajectory.Resolve(msg.waypoints)
    }
    else {
      resolved.waypoints = new trajectory_msgs.msg.JointTrajectory()
    }

    if (msg.sampling_frequency !== undefined) {
      resolved.sampling_frequency = msg.sampling_frequency;
    }
    else {
      resolved.sampling_frequency = 0.0
    }

    if (msg.n_gridpoints !== undefined) {
      resolved.n_gridpoints = msg.n_gridpoints;
    }
    else {
      resolved.n_gridpoints = 0
    }

    if (msg.plot !== undefined) {
      resolved.plot = msg.plot;
    }
    else {
      resolved.plot = false
    }

    return resolved;
    }
};

class GenerateTrajectoryResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory = null;
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory')) {
        this.trajectory = initObj.trajectory
      }
      else {
        this.trajectory = new trajectory_msgs.msg.JointTrajectory();
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GenerateTrajectoryResponse
    // Serialize message field [trajectory]
    bufferOffset = trajectory_msgs.msg.JointTrajectory.serialize(obj.trajectory, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GenerateTrajectoryResponse
    let len;
    let data = new GenerateTrajectoryResponse(null);
    // Deserialize message field [trajectory]
    data.trajectory = trajectory_msgs.msg.JointTrajectory.deserialize(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.JointTrajectory.getMessageSize(object.trajectory);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'topp_ros/GenerateTrajectoryResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2bb9e8a7d94d07c734870b8a7b06730c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    trajectory_msgs/JointTrajectory     trajectory
    bool                                success
    
    
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
    const resolved = new GenerateTrajectoryResponse(null);
    if (msg.trajectory !== undefined) {
      resolved.trajectory = trajectory_msgs.msg.JointTrajectory.Resolve(msg.trajectory)
    }
    else {
      resolved.trajectory = new trajectory_msgs.msg.JointTrajectory()
    }

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
  Request: GenerateTrajectoryRequest,
  Response: GenerateTrajectoryResponse,
  md5sum() { return 'bf5d8c7f17b854de72ace6f948a3f238'; },
  datatype() { return 'topp_ros/GenerateTrajectory'; }
};
