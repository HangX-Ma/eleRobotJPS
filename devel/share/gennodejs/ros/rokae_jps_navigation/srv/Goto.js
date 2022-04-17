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

class GotoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal_pose = null;
      this.ifback = null;
    }
    else {
      if (initObj.hasOwnProperty('goal_pose')) {
        this.goal_pose = initObj.goal_pose
      }
      else {
        this.goal_pose = [];
      }
      if (initObj.hasOwnProperty('ifback')) {
        this.ifback = initObj.ifback
      }
      else {
        this.ifback = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GotoRequest
    // Serialize message field [goal_pose]
    // Serialize the length for message field [goal_pose]
    bufferOffset = _serializer.uint32(obj.goal_pose.length, buffer, bufferOffset);
    obj.goal_pose.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [ifback]
    bufferOffset = _serializer.bool(obj.ifback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GotoRequest
    let len;
    let data = new GotoRequest(null);
    // Deserialize message field [goal_pose]
    // Deserialize array length for message field [goal_pose]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.goal_pose = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.goal_pose[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [ifback]
    data.ifback = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 56 * object.goal_pose.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/GotoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '62c5f8fdc5b9c61ae6bc9f992786c85d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose[] goal_pose
    bool ifback
    
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
    const resolved = new GotoRequest(null);
    if (msg.goal_pose !== undefined) {
      resolved.goal_pose = new Array(msg.goal_pose.length);
      for (let i = 0; i < resolved.goal_pose.length; ++i) {
        resolved.goal_pose[i] = geometry_msgs.msg.Pose.Resolve(msg.goal_pose[i]);
      }
    }
    else {
      resolved.goal_pose = []
    }

    if (msg.ifback !== undefined) {
      resolved.ifback = msg.ifback;
    }
    else {
      resolved.ifback = false
    }

    return resolved;
    }
};

class GotoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message = null;
      this.success = null;
      this.px = null;
      this.py = null;
      this.pz = null;
      this.pos = null;
      this.vel = null;
      this.acc = null;
      this.t = null;
      this.back_pos = null;
      this.back_vel = null;
      this.back_acc = null;
      this.back_t = null;
    }
    else {
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('px')) {
        this.px = initObj.px
      }
      else {
        this.px = [];
      }
      if (initObj.hasOwnProperty('py')) {
        this.py = initObj.py
      }
      else {
        this.py = [];
      }
      if (initObj.hasOwnProperty('pz')) {
        this.pz = initObj.pz
      }
      else {
        this.pz = [];
      }
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
      if (initObj.hasOwnProperty('back_pos')) {
        this.back_pos = initObj.back_pos
      }
      else {
        this.back_pos = [];
      }
      if (initObj.hasOwnProperty('back_vel')) {
        this.back_vel = initObj.back_vel
      }
      else {
        this.back_vel = [];
      }
      if (initObj.hasOwnProperty('back_acc')) {
        this.back_acc = initObj.back_acc
      }
      else {
        this.back_acc = [];
      }
      if (initObj.hasOwnProperty('back_t')) {
        this.back_t = initObj.back_t
      }
      else {
        this.back_t = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GotoResponse
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [px]
    bufferOffset = _arraySerializer.float64(obj.px, buffer, bufferOffset, null);
    // Serialize message field [py]
    bufferOffset = _arraySerializer.float64(obj.py, buffer, bufferOffset, null);
    // Serialize message field [pz]
    bufferOffset = _arraySerializer.float64(obj.pz, buffer, bufferOffset, null);
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, null);
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, null);
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, null);
    // Serialize message field [t]
    bufferOffset = _arraySerializer.float64(obj.t, buffer, bufferOffset, null);
    // Serialize message field [back_pos]
    bufferOffset = _arraySerializer.float64(obj.back_pos, buffer, bufferOffset, null);
    // Serialize message field [back_vel]
    bufferOffset = _arraySerializer.float64(obj.back_vel, buffer, bufferOffset, null);
    // Serialize message field [back_acc]
    bufferOffset = _arraySerializer.float64(obj.back_acc, buffer, bufferOffset, null);
    // Serialize message field [back_t]
    bufferOffset = _arraySerializer.float64(obj.back_t, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GotoResponse
    let len;
    let data = new GotoResponse(null);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [px]
    data.px = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [py]
    data.py = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [pz]
    data.pz = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [t]
    data.t = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [back_pos]
    data.back_pos = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [back_vel]
    data.back_vel = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [back_acc]
    data.back_acc = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [back_t]
    data.back_t = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.message.length;
    length += 8 * object.px.length;
    length += 8 * object.py.length;
    length += 8 * object.pz.length;
    length += 8 * object.pos.length;
    length += 8 * object.vel.length;
    length += 8 * object.acc.length;
    length += 8 * object.t.length;
    length += 8 * object.back_pos.length;
    length += 8 * object.back_vel.length;
    length += 8 * object.back_acc.length;
    length += 8 * object.back_t.length;
    return length + 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rokae_jps_navigation/GotoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d1f4162acc575e9ecdf795508e248a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string message
    bool success
    float64[] px
    float64[] py
    float64[] pz
    float64[] pos
    float64[] vel
    float64[] acc
    float64[] t
    float64[] back_pos
    float64[] back_vel
    float64[] back_acc
    float64[] back_t
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GotoResponse(null);
    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.px !== undefined) {
      resolved.px = msg.px;
    }
    else {
      resolved.px = []
    }

    if (msg.py !== undefined) {
      resolved.py = msg.py;
    }
    else {
      resolved.py = []
    }

    if (msg.pz !== undefined) {
      resolved.pz = msg.pz;
    }
    else {
      resolved.pz = []
    }

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

    if (msg.back_pos !== undefined) {
      resolved.back_pos = msg.back_pos;
    }
    else {
      resolved.back_pos = []
    }

    if (msg.back_vel !== undefined) {
      resolved.back_vel = msg.back_vel;
    }
    else {
      resolved.back_vel = []
    }

    if (msg.back_acc !== undefined) {
      resolved.back_acc = msg.back_acc;
    }
    else {
      resolved.back_acc = []
    }

    if (msg.back_t !== undefined) {
      resolved.back_t = msg.back_t;
    }
    else {
      resolved.back_t = []
    }

    return resolved;
    }
};

module.exports = {
  Request: GotoRequest,
  Response: GotoResponse,
  md5sum() { return 'db7c9c254c9a1585396c175f27b28c32'; },
  datatype() { return 'rokae_jps_navigation/Goto'; }
};
