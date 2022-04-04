// Auto-generated. Do not edit!

// (in-package rokae_jps_navigation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class CheckCollisionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path_pose = null;
      this.prev_joints = null;
      this.ifVerbose = null;
    }
    else {
      if (initObj.hasOwnProperty('path_pose')) {
        this.path_pose = initObj.path_pose
      }
      else {
        this.path_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('prev_joints')) {
        this.prev_joints = initObj.prev_joints
      }
      else {
        this.prev_joints = [];
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
    // Serializes a message object of type CheckCollisionRequest
    // Serialize message field [path_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.path_pose, buffer, bufferOffset);
    // Serialize message field [prev_joints]
    bufferOffset = _arraySerializer.float32(obj.prev_joints, buffer, bufferOffset, null);
    // Serialize message field [ifVerbose]
    bufferOffset = _serializer.bool(obj.ifVerbose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckCollisionRequest
    let len;
    let data = new CheckCollisionRequest(null);
    // Deserialize message field [path_pose]
    data.path_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [prev_joints]
    data.prev_joints = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [ifVerbose]
    data.ifVerbose = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.prev_joints.length;
    return length + 61;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/CheckCollisionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bd89d9349701f68069b0f63564cad46';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose path_pose
    float32[] prev_joints
    bool ifVerbose
    
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
    const resolved = new CheckCollisionRequest(null);
    if (msg.path_pose !== undefined) {
      resolved.path_pose = geometry_msgs.msg.Pose.Resolve(msg.path_pose)
    }
    else {
      resolved.path_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.prev_joints !== undefined) {
      resolved.prev_joints = msg.prev_joints;
    }
    else {
      resolved.prev_joints = []
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

class CheckCollisionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isCollide = null;
      this.getSolution = null;
      this.curr_joints = null;
    }
    else {
      if (initObj.hasOwnProperty('isCollide')) {
        this.isCollide = initObj.isCollide
      }
      else {
        this.isCollide = false;
      }
      if (initObj.hasOwnProperty('getSolution')) {
        this.getSolution = initObj.getSolution
      }
      else {
        this.getSolution = false;
      }
      if (initObj.hasOwnProperty('curr_joints')) {
        this.curr_joints = initObj.curr_joints
      }
      else {
        this.curr_joints = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CheckCollisionResponse
    // Serialize message field [isCollide]
    bufferOffset = _serializer.bool(obj.isCollide, buffer, bufferOffset);
    // Serialize message field [getSolution]
    bufferOffset = _serializer.bool(obj.getSolution, buffer, bufferOffset);
    // Serialize message field [curr_joints]
    bufferOffset = _arraySerializer.float32(obj.curr_joints, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CheckCollisionResponse
    let len;
    let data = new CheckCollisionResponse(null);
    // Deserialize message field [isCollide]
    data.isCollide = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [getSolution]
    data.getSolution = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [curr_joints]
    data.curr_joints = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.curr_joints.length;
    return length + 6;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/CheckCollisionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '249a06f30727d2046b37e953d9335a81';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isCollide
    bool getSolution
    float32[] curr_joints
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CheckCollisionResponse(null);
    if (msg.isCollide !== undefined) {
      resolved.isCollide = msg.isCollide;
    }
    else {
      resolved.isCollide = false
    }

    if (msg.getSolution !== undefined) {
      resolved.getSolution = msg.getSolution;
    }
    else {
      resolved.getSolution = false
    }

    if (msg.curr_joints !== undefined) {
      resolved.curr_joints = msg.curr_joints;
    }
    else {
      resolved.curr_joints = []
    }

    return resolved;
    }
};

module.exports = {
  Request: CheckCollisionRequest,
  Response: CheckCollisionResponse,
  md5sum() { return '9553736970fcba3d957b6a5220bb6872'; },
  datatype() { return 'rokae_jps_navigation/CheckCollision'; }
};
