#pragma once
// MESSAGE MOTOR_COMMAND PACKING

#define MAVLINK_MSG_ID_MOTOR_COMMAND 12004


typedef struct __mavlink_motor_command_t {
 float target_value; /*<  Target value (units depend on control_mode)*/
 uint8_t motor_id; /*<  Motor ID (1-255)*/
 uint8_t control_mode; /*<  Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)*/
 uint8_t enable; /*<  Enable motor (0=disable, 1=enable)*/
} mavlink_motor_command_t;

#define MAVLINK_MSG_ID_MOTOR_COMMAND_LEN 7
#define MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN 7
#define MAVLINK_MSG_ID_12004_LEN 7
#define MAVLINK_MSG_ID_12004_MIN_LEN 7

#define MAVLINK_MSG_ID_MOTOR_COMMAND_CRC 212
#define MAVLINK_MSG_ID_12004_CRC 212



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTOR_COMMAND { \
    12004, \
    "MOTOR_COMMAND", \
    4, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_motor_command_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_motor_command_t, control_mode) }, \
         { "target_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_command_t, target_value) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_motor_command_t, enable) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTOR_COMMAND { \
    "MOTOR_COMMAND", \
    4, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_motor_command_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_motor_command_t, control_mode) }, \
         { "target_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motor_command_t, target_value) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_motor_command_t, enable) }, \
         } \
}
#endif

/**
 * @brief Pack a motor_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)
 * @param target_value  Target value (units depend on control_mode)
 * @param enable  Enable motor (0=disable, 1=enable)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, float target_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
    mavlink_motor_command_t packet;
    packet.target_value = target_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
}

/**
 * @brief Pack a motor_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)
 * @param target_value  Target value (units depend on control_mode)
 * @param enable  Enable motor (0=disable, 1=enable)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, float target_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
    mavlink_motor_command_t packet;
    packet.target_value = target_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif
}

/**
 * @brief Pack a motor_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)
 * @param target_value  Target value (units depend on control_mode)
 * @param enable  Enable motor (0=disable, 1=enable)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motor_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint8_t control_mode,float target_value,uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#else
    mavlink_motor_command_t packet;
    packet.target_value = target_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTOR_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
}

/**
 * @brief Encode a motor_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
    return mavlink_msg_motor_command_pack(system_id, component_id, msg, motor_command->motor_id, motor_command->control_mode, motor_command->target_value, motor_command->enable);
}

/**
 * @brief Encode a motor_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
    return mavlink_msg_motor_command_pack_chan(system_id, component_id, chan, msg, motor_command->motor_id, motor_command->control_mode, motor_command->target_value, motor_command->enable);
}

/**
 * @brief Encode a motor_command struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param motor_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motor_command_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_motor_command_t* motor_command)
{
    return mavlink_msg_motor_command_pack_status(system_id, component_id, _status, msg,  motor_command->motor_id, motor_command->control_mode, motor_command->target_value, motor_command->enable);
}

/**
 * @brief Send a motor_command message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)
 * @param target_value  Target value (units depend on control_mode)
 * @param enable  Enable motor (0=disable, 1=enable)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motor_command_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t control_mode, float target_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTOR_COMMAND_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    mavlink_motor_command_t packet;
    packet.target_value = target_value;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.enable = enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#endif
}

/**
 * @brief Send a motor_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motor_command_send_struct(mavlink_channel_t chan, const mavlink_motor_command_t* motor_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motor_command_send(chan, motor_command->motor_id, motor_command->control_mode, motor_command->target_value, motor_command->enable);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)motor_command, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTOR_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motor_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint8_t control_mode, float target_value, uint8_t enable)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, target_value);
    _mav_put_uint8_t(buf, 4, motor_id);
    _mav_put_uint8_t(buf, 5, control_mode);
    _mav_put_uint8_t(buf, 6, enable);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, buf, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#else
    mavlink_motor_command_t *packet = (mavlink_motor_command_t *)msgbuf;
    packet->target_value = target_value;
    packet->motor_id = motor_id;
    packet->control_mode = control_mode;
    packet->enable = enable;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTOR_COMMAND, (const char *)packet, MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN, MAVLINK_MSG_ID_MOTOR_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTOR_COMMAND UNPACKING


/**
 * @brief Get field motor_id from motor_command message
 *
 * @return  Motor ID (1-255)
 */
static inline uint8_t mavlink_msg_motor_command_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field control_mode from motor_command message
 *
 * @return  Control mode (0=position, 1=velocity, 2=current, 3=duty_cycle)
 */
static inline uint8_t mavlink_msg_motor_command_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field target_value from motor_command message
 *
 * @return  Target value (units depend on control_mode)
 */
static inline float mavlink_msg_motor_command_get_target_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field enable from motor_command message
 *
 * @return  Enable motor (0=disable, 1=enable)
 */
static inline uint8_t mavlink_msg_motor_command_get_enable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Decode a motor_command message into a struct
 *
 * @param msg The message to decode
 * @param motor_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_motor_command_decode(const mavlink_message_t* msg, mavlink_motor_command_t* motor_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motor_command->target_value = mavlink_msg_motor_command_get_target_value(msg);
    motor_command->motor_id = mavlink_msg_motor_command_get_motor_id(msg);
    motor_command->control_mode = mavlink_msg_motor_command_get_control_mode(msg);
    motor_command->enable = mavlink_msg_motor_command_get_enable(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTOR_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_MOTOR_COMMAND_LEN;
        memset(motor_command, 0, MAVLINK_MSG_ID_MOTOR_COMMAND_LEN);
    memcpy(motor_command, _MAV_PAYLOAD(msg), len);
#endif
}
