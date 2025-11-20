#pragma once
// MESSAGE RS485_MOTOR_STATUS PACKING

#define MAVLINK_MSG_ID_RS485_MOTOR_STATUS 12005


typedef struct __mavlink_rs485_motor_status_t {
 float current_position_rotations; /*<  Current position in rotations*/
 float current_velocity_rps; /*<  Current velocity in revolutions per second*/
 float target_velocity_rps; /*<  Target velocity in revolutions per second*/
 float acceleration_rps2; /*<  Acceleration setting in rev/s^2*/
 uint32_t timestamp_ms; /*<  Timestamp in milliseconds*/
 uint8_t motor_id; /*<  Motor ID (30-49 for RS485 motors)*/
 uint8_t device_id; /*<  RS485 board device ID (DIP switch setting)*/
 uint8_t motor_index; /*<  Motor index on board (0-2 for 3-motor boards)*/
 uint8_t control_mode; /*<  Control mode (0=position, 1=velocity)*/
 uint8_t status; /*<  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)*/
 uint8_t error_code; /*<  RS485 protocol error code (0=no error)*/
} mavlink_rs485_motor_status_t;

#define MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN 26
#define MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN 26
#define MAVLINK_MSG_ID_12005_LEN 26
#define MAVLINK_MSG_ID_12005_MIN_LEN 26

#define MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC 45
#define MAVLINK_MSG_ID_12005_CRC 45



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RS485_MOTOR_STATUS { \
    12005, \
    "RS485_MOTOR_STATUS", \
    11, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_rs485_motor_status_t, motor_id) }, \
         { "device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_rs485_motor_status_t, device_id) }, \
         { "motor_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_rs485_motor_status_t, motor_index) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_rs485_motor_status_t, control_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_rs485_motor_status_t, status) }, \
         { "error_code", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_rs485_motor_status_t, error_code) }, \
         { "current_position_rotations", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rs485_motor_status_t, current_position_rotations) }, \
         { "current_velocity_rps", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rs485_motor_status_t, current_velocity_rps) }, \
         { "target_velocity_rps", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rs485_motor_status_t, target_velocity_rps) }, \
         { "acceleration_rps2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rs485_motor_status_t, acceleration_rps2) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_rs485_motor_status_t, timestamp_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RS485_MOTOR_STATUS { \
    "RS485_MOTOR_STATUS", \
    11, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_rs485_motor_status_t, motor_id) }, \
         { "device_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_rs485_motor_status_t, device_id) }, \
         { "motor_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_rs485_motor_status_t, motor_index) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_rs485_motor_status_t, control_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_rs485_motor_status_t, status) }, \
         { "error_code", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_rs485_motor_status_t, error_code) }, \
         { "current_position_rotations", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rs485_motor_status_t, current_position_rotations) }, \
         { "current_velocity_rps", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rs485_motor_status_t, current_velocity_rps) }, \
         { "target_velocity_rps", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rs485_motor_status_t, target_velocity_rps) }, \
         { "acceleration_rps2", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rs485_motor_status_t, acceleration_rps2) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_rs485_motor_status_t, timestamp_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a rs485_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param device_id  RS485 board device ID (DIP switch setting)
 * @param motor_index  Motor index on board (0-2 for 3-motor boards)
 * @param control_mode  Control mode (0=position, 1=velocity)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param error_code  RS485 protocol error code (0=no error)
 * @param current_position_rotations  Current position in rotations
 * @param current_velocity_rps  Current velocity in revolutions per second
 * @param target_velocity_rps  Target velocity in revolutions per second
 * @param acceleration_rps2  Acceleration setting in rev/s^2
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_motor_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t device_id, uint8_t motor_index, uint8_t control_mode, uint8_t status, uint8_t error_code, float current_position_rotations, float current_velocity_rps, float target_velocity_rps, float acceleration_rps2, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rotations);
    _mav_put_float(buf, 4, current_velocity_rps);
    _mav_put_float(buf, 8, target_velocity_rps);
    _mav_put_float(buf, 12, acceleration_rps2);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, device_id);
    _mav_put_uint8_t(buf, 22, motor_index);
    _mav_put_uint8_t(buf, 23, control_mode);
    _mav_put_uint8_t(buf, 24, status);
    _mav_put_uint8_t(buf, 25, error_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#else
    mavlink_rs485_motor_status_t packet;
    packet.current_position_rotations = current_position_rotations;
    packet.current_velocity_rps = current_velocity_rps;
    packet.target_velocity_rps = target_velocity_rps;
    packet.acceleration_rps2 = acceleration_rps2;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.device_id = device_id;
    packet.motor_index = motor_index;
    packet.control_mode = control_mode;
    packet.status = status;
    packet.error_code = error_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_MOTOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
}

/**
 * @brief Pack a rs485_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param device_id  RS485 board device ID (DIP switch setting)
 * @param motor_index  Motor index on board (0-2 for 3-motor boards)
 * @param control_mode  Control mode (0=position, 1=velocity)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param error_code  RS485 protocol error code (0=no error)
 * @param current_position_rotations  Current position in rotations
 * @param current_velocity_rps  Current velocity in revolutions per second
 * @param target_velocity_rps  Target velocity in revolutions per second
 * @param acceleration_rps2  Acceleration setting in rev/s^2
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_motor_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t device_id, uint8_t motor_index, uint8_t control_mode, uint8_t status, uint8_t error_code, float current_position_rotations, float current_velocity_rps, float target_velocity_rps, float acceleration_rps2, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rotations);
    _mav_put_float(buf, 4, current_velocity_rps);
    _mav_put_float(buf, 8, target_velocity_rps);
    _mav_put_float(buf, 12, acceleration_rps2);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, device_id);
    _mav_put_uint8_t(buf, 22, motor_index);
    _mav_put_uint8_t(buf, 23, control_mode);
    _mav_put_uint8_t(buf, 24, status);
    _mav_put_uint8_t(buf, 25, error_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#else
    mavlink_rs485_motor_status_t packet;
    packet.current_position_rotations = current_position_rotations;
    packet.current_velocity_rps = current_velocity_rps;
    packet.target_velocity_rps = target_velocity_rps;
    packet.acceleration_rps2 = acceleration_rps2;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.device_id = device_id;
    packet.motor_index = motor_index;
    packet.control_mode = control_mode;
    packet.status = status;
    packet.error_code = error_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_MOTOR_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#endif
}

/**
 * @brief Pack a rs485_motor_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param device_id  RS485 board device ID (DIP switch setting)
 * @param motor_index  Motor index on board (0-2 for 3-motor boards)
 * @param control_mode  Control mode (0=position, 1=velocity)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param error_code  RS485 protocol error code (0=no error)
 * @param current_position_rotations  Current position in rotations
 * @param current_velocity_rps  Current velocity in revolutions per second
 * @param target_velocity_rps  Target velocity in revolutions per second
 * @param acceleration_rps2  Acceleration setting in rev/s^2
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_motor_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint8_t device_id,uint8_t motor_index,uint8_t control_mode,uint8_t status,uint8_t error_code,float current_position_rotations,float current_velocity_rps,float target_velocity_rps,float acceleration_rps2,uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rotations);
    _mav_put_float(buf, 4, current_velocity_rps);
    _mav_put_float(buf, 8, target_velocity_rps);
    _mav_put_float(buf, 12, acceleration_rps2);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, device_id);
    _mav_put_uint8_t(buf, 22, motor_index);
    _mav_put_uint8_t(buf, 23, control_mode);
    _mav_put_uint8_t(buf, 24, status);
    _mav_put_uint8_t(buf, 25, error_code);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#else
    mavlink_rs485_motor_status_t packet;
    packet.current_position_rotations = current_position_rotations;
    packet.current_velocity_rps = current_velocity_rps;
    packet.target_velocity_rps = target_velocity_rps;
    packet.acceleration_rps2 = acceleration_rps2;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.device_id = device_id;
    packet.motor_index = motor_index;
    packet.control_mode = control_mode;
    packet.status = status;
    packet.error_code = error_code;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_MOTOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
}

/**
 * @brief Encode a rs485_motor_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rs485_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_motor_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rs485_motor_status_t* rs485_motor_status)
{
    return mavlink_msg_rs485_motor_status_pack(system_id, component_id, msg, rs485_motor_status->motor_id, rs485_motor_status->device_id, rs485_motor_status->motor_index, rs485_motor_status->control_mode, rs485_motor_status->status, rs485_motor_status->error_code, rs485_motor_status->current_position_rotations, rs485_motor_status->current_velocity_rps, rs485_motor_status->target_velocity_rps, rs485_motor_status->acceleration_rps2, rs485_motor_status->timestamp_ms);
}

/**
 * @brief Encode a rs485_motor_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rs485_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_motor_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rs485_motor_status_t* rs485_motor_status)
{
    return mavlink_msg_rs485_motor_status_pack_chan(system_id, component_id, chan, msg, rs485_motor_status->motor_id, rs485_motor_status->device_id, rs485_motor_status->motor_index, rs485_motor_status->control_mode, rs485_motor_status->status, rs485_motor_status->error_code, rs485_motor_status->current_position_rotations, rs485_motor_status->current_velocity_rps, rs485_motor_status->target_velocity_rps, rs485_motor_status->acceleration_rps2, rs485_motor_status->timestamp_ms);
}

/**
 * @brief Encode a rs485_motor_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rs485_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_motor_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rs485_motor_status_t* rs485_motor_status)
{
    return mavlink_msg_rs485_motor_status_pack_status(system_id, component_id, _status, msg,  rs485_motor_status->motor_id, rs485_motor_status->device_id, rs485_motor_status->motor_index, rs485_motor_status->control_mode, rs485_motor_status->status, rs485_motor_status->error_code, rs485_motor_status->current_position_rotations, rs485_motor_status->current_velocity_rps, rs485_motor_status->target_velocity_rps, rs485_motor_status->acceleration_rps2, rs485_motor_status->timestamp_ms);
}

/**
 * @brief Send a rs485_motor_status message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param device_id  RS485 board device ID (DIP switch setting)
 * @param motor_index  Motor index on board (0-2 for 3-motor boards)
 * @param control_mode  Control mode (0=position, 1=velocity)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 * @param error_code  RS485 protocol error code (0=no error)
 * @param current_position_rotations  Current position in rotations
 * @param current_velocity_rps  Current velocity in revolutions per second
 * @param target_velocity_rps  Target velocity in revolutions per second
 * @param acceleration_rps2  Acceleration setting in rev/s^2
 * @param timestamp_ms  Timestamp in milliseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rs485_motor_status_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t device_id, uint8_t motor_index, uint8_t control_mode, uint8_t status, uint8_t error_code, float current_position_rotations, float current_velocity_rps, float target_velocity_rps, float acceleration_rps2, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, current_position_rotations);
    _mav_put_float(buf, 4, current_velocity_rps);
    _mav_put_float(buf, 8, target_velocity_rps);
    _mav_put_float(buf, 12, acceleration_rps2);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, device_id);
    _mav_put_uint8_t(buf, 22, motor_index);
    _mav_put_uint8_t(buf, 23, control_mode);
    _mav_put_uint8_t(buf, 24, status);
    _mav_put_uint8_t(buf, 25, error_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_MOTOR_STATUS, buf, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
#else
    mavlink_rs485_motor_status_t packet;
    packet.current_position_rotations = current_position_rotations;
    packet.current_velocity_rps = current_velocity_rps;
    packet.target_velocity_rps = target_velocity_rps;
    packet.acceleration_rps2 = acceleration_rps2;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.device_id = device_id;
    packet.motor_index = motor_index;
    packet.control_mode = control_mode;
    packet.status = status;
    packet.error_code = error_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_MOTOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a rs485_motor_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rs485_motor_status_send_struct(mavlink_channel_t chan, const mavlink_rs485_motor_status_t* rs485_motor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rs485_motor_status_send(chan, rs485_motor_status->motor_id, rs485_motor_status->device_id, rs485_motor_status->motor_index, rs485_motor_status->control_mode, rs485_motor_status->status, rs485_motor_status->error_code, rs485_motor_status->current_position_rotations, rs485_motor_status->current_velocity_rps, rs485_motor_status->target_velocity_rps, rs485_motor_status->acceleration_rps2, rs485_motor_status->timestamp_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_MOTOR_STATUS, (const char *)rs485_motor_status, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rs485_motor_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint8_t device_id, uint8_t motor_index, uint8_t control_mode, uint8_t status, uint8_t error_code, float current_position_rotations, float current_velocity_rps, float target_velocity_rps, float acceleration_rps2, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, current_position_rotations);
    _mav_put_float(buf, 4, current_velocity_rps);
    _mav_put_float(buf, 8, target_velocity_rps);
    _mav_put_float(buf, 12, acceleration_rps2);
    _mav_put_uint32_t(buf, 16, timestamp_ms);
    _mav_put_uint8_t(buf, 20, motor_id);
    _mav_put_uint8_t(buf, 21, device_id);
    _mav_put_uint8_t(buf, 22, motor_index);
    _mav_put_uint8_t(buf, 23, control_mode);
    _mav_put_uint8_t(buf, 24, status);
    _mav_put_uint8_t(buf, 25, error_code);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_MOTOR_STATUS, buf, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
#else
    mavlink_rs485_motor_status_t *packet = (mavlink_rs485_motor_status_t *)msgbuf;
    packet->current_position_rotations = current_position_rotations;
    packet->current_velocity_rps = current_velocity_rps;
    packet->target_velocity_rps = target_velocity_rps;
    packet->acceleration_rps2 = acceleration_rps2;
    packet->timestamp_ms = timestamp_ms;
    packet->motor_id = motor_id;
    packet->device_id = device_id;
    packet->motor_index = motor_index;
    packet->control_mode = control_mode;
    packet->status = status;
    packet->error_code = error_code;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_MOTOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE RS485_MOTOR_STATUS UNPACKING


/**
 * @brief Get field motor_id from rs485_motor_status message
 *
 * @return  Motor ID (30-49 for RS485 motors)
 */
static inline uint8_t mavlink_msg_rs485_motor_status_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field device_id from rs485_motor_status message
 *
 * @return  RS485 board device ID (DIP switch setting)
 */
static inline uint8_t mavlink_msg_rs485_motor_status_get_device_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field motor_index from rs485_motor_status message
 *
 * @return  Motor index on board (0-2 for 3-motor boards)
 */
static inline uint8_t mavlink_msg_rs485_motor_status_get_motor_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field control_mode from rs485_motor_status message
 *
 * @return  Control mode (0=position, 1=velocity)
 */
static inline uint8_t mavlink_msg_rs485_motor_status_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field status from rs485_motor_status message
 *
 * @return  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED)
 */
static inline uint8_t mavlink_msg_rs485_motor_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field error_code from rs485_motor_status message
 *
 * @return  RS485 protocol error code (0=no error)
 */
static inline uint8_t mavlink_msg_rs485_motor_status_get_error_code(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field current_position_rotations from rs485_motor_status message
 *
 * @return  Current position in rotations
 */
static inline float mavlink_msg_rs485_motor_status_get_current_position_rotations(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current_velocity_rps from rs485_motor_status message
 *
 * @return  Current velocity in revolutions per second
 */
static inline float mavlink_msg_rs485_motor_status_get_current_velocity_rps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field target_velocity_rps from rs485_motor_status message
 *
 * @return  Target velocity in revolutions per second
 */
static inline float mavlink_msg_rs485_motor_status_get_target_velocity_rps(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field acceleration_rps2 from rs485_motor_status message
 *
 * @return  Acceleration setting in rev/s^2
 */
static inline float mavlink_msg_rs485_motor_status_get_acceleration_rps2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field timestamp_ms from rs485_motor_status message
 *
 * @return  Timestamp in milliseconds
 */
static inline uint32_t mavlink_msg_rs485_motor_status_get_timestamp_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a rs485_motor_status message into a struct
 *
 * @param msg The message to decode
 * @param rs485_motor_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_rs485_motor_status_decode(const mavlink_message_t* msg, mavlink_rs485_motor_status_t* rs485_motor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rs485_motor_status->current_position_rotations = mavlink_msg_rs485_motor_status_get_current_position_rotations(msg);
    rs485_motor_status->current_velocity_rps = mavlink_msg_rs485_motor_status_get_current_velocity_rps(msg);
    rs485_motor_status->target_velocity_rps = mavlink_msg_rs485_motor_status_get_target_velocity_rps(msg);
    rs485_motor_status->acceleration_rps2 = mavlink_msg_rs485_motor_status_get_acceleration_rps2(msg);
    rs485_motor_status->timestamp_ms = mavlink_msg_rs485_motor_status_get_timestamp_ms(msg);
    rs485_motor_status->motor_id = mavlink_msg_rs485_motor_status_get_motor_id(msg);
    rs485_motor_status->device_id = mavlink_msg_rs485_motor_status_get_device_id(msg);
    rs485_motor_status->motor_index = mavlink_msg_rs485_motor_status_get_motor_index(msg);
    rs485_motor_status->control_mode = mavlink_msg_rs485_motor_status_get_control_mode(msg);
    rs485_motor_status->status = mavlink_msg_rs485_motor_status_get_status(msg);
    rs485_motor_status->error_code = mavlink_msg_rs485_motor_status_get_error_code(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN;
        memset(rs485_motor_status, 0, MAVLINK_MSG_ID_RS485_MOTOR_STATUS_LEN);
    memcpy(rs485_motor_status, _MAV_PAYLOAD(msg), len);
#endif
}
