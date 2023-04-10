// Auto-generated. Do not edit!

// (in-package ur_planning.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class grasp_poseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.grasppose_x = null;
      this.grasppose_y = null;
      this.grasppose_z = null;
      this.grasppose_R = null;
      this.grasppose_P = null;
      this.grasppose_Y = null;
    }
    else {
      if (initObj.hasOwnProperty('grasppose_x')) {
        this.grasppose_x = initObj.grasppose_x
      }
      else {
        this.grasppose_x = 0.0;
      }
      if (initObj.hasOwnProperty('grasppose_y')) {
        this.grasppose_y = initObj.grasppose_y
      }
      else {
        this.grasppose_y = 0.0;
      }
      if (initObj.hasOwnProperty('grasppose_z')) {
        this.grasppose_z = initObj.grasppose_z
      }
      else {
        this.grasppose_z = 0.0;
      }
      if (initObj.hasOwnProperty('grasppose_R')) {
        this.grasppose_R = initObj.grasppose_R
      }
      else {
        this.grasppose_R = 0.0;
      }
      if (initObj.hasOwnProperty('grasppose_P')) {
        this.grasppose_P = initObj.grasppose_P
      }
      else {
        this.grasppose_P = 0.0;
      }
      if (initObj.hasOwnProperty('grasppose_Y')) {
        this.grasppose_Y = initObj.grasppose_Y
      }
      else {
        this.grasppose_Y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type grasp_poseRequest
    // Serialize message field [grasppose_x]
    bufferOffset = _serializer.float64(obj.grasppose_x, buffer, bufferOffset);
    // Serialize message field [grasppose_y]
    bufferOffset = _serializer.float64(obj.grasppose_y, buffer, bufferOffset);
    // Serialize message field [grasppose_z]
    bufferOffset = _serializer.float64(obj.grasppose_z, buffer, bufferOffset);
    // Serialize message field [grasppose_R]
    bufferOffset = _serializer.float64(obj.grasppose_R, buffer, bufferOffset);
    // Serialize message field [grasppose_P]
    bufferOffset = _serializer.float64(obj.grasppose_P, buffer, bufferOffset);
    // Serialize message field [grasppose_Y]
    bufferOffset = _serializer.float64(obj.grasppose_Y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type grasp_poseRequest
    let len;
    let data = new grasp_poseRequest(null);
    // Deserialize message field [grasppose_x]
    data.grasppose_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasppose_y]
    data.grasppose_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasppose_z]
    data.grasppose_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasppose_R]
    data.grasppose_R = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasppose_P]
    data.grasppose_P = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [grasppose_Y]
    data.grasppose_Y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_planning/grasp_poseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '750783d28a717b2d8f54ebfe11d1d013';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 grasppose_x
    float64 grasppose_y
    float64 grasppose_z
    float64 grasppose_R
    float64 grasppose_P
    float64 grasppose_Y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new grasp_poseRequest(null);
    if (msg.grasppose_x !== undefined) {
      resolved.grasppose_x = msg.grasppose_x;
    }
    else {
      resolved.grasppose_x = 0.0
    }

    if (msg.grasppose_y !== undefined) {
      resolved.grasppose_y = msg.grasppose_y;
    }
    else {
      resolved.grasppose_y = 0.0
    }

    if (msg.grasppose_z !== undefined) {
      resolved.grasppose_z = msg.grasppose_z;
    }
    else {
      resolved.grasppose_z = 0.0
    }

    if (msg.grasppose_R !== undefined) {
      resolved.grasppose_R = msg.grasppose_R;
    }
    else {
      resolved.grasppose_R = 0.0
    }

    if (msg.grasppose_P !== undefined) {
      resolved.grasppose_P = msg.grasppose_P;
    }
    else {
      resolved.grasppose_P = 0.0
    }

    if (msg.grasppose_Y !== undefined) {
      resolved.grasppose_Y = msg.grasppose_Y;
    }
    else {
      resolved.grasppose_Y = 0.0
    }

    return resolved;
    }
};

class grasp_poseResponse {
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
    // Serializes a message object of type grasp_poseResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type grasp_poseResponse
    let len;
    let data = new grasp_poseResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_planning/grasp_poseResponse';
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
    const resolved = new grasp_poseResponse(null);
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
  Request: grasp_poseRequest,
  Response: grasp_poseResponse,
  md5sum() { return '8a5978aa25b0b059ee08ab4584d6c721'; },
  datatype() { return 'ur_planning/grasp_pose'; }
};
