#pragma once
// MESSAGE DC_MOTOR_CONTROL PACKING

#define MAVLINK_MSG_ID_DC_MOTOR_CONTROL 12002


typedef struct __mavlink_dc_motor_control_t {
 float target_value; /*<  Target value (PWM duty -1.0 to 1.0, speed in rad/s, or position in rad)*/
 float speed_limit_rad_s; /*<  Speed limit in rad/s (0 = use motor default limit)*/
 float acceleration_limit_rad_s2; /*<  Acceleration limit in rad/s^2 (0 = use motor default limit)*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 uint8_t motor_id; /*<  Motor ID (1-255)*/
 uint8_t control_mode; /*<  Control mode (0=PWM/Open Loop, 1=Speed Control, 2=Position Control, 3=Disabled)*/
} mavlink_dc_motor_control_t;

#define MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN 16
#define MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN 16
#define MAVLINK_MSG_ID_12002_LEN 16
#define MAVLINK_MSG_ID_12002_MIN_LEN 16

#define MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC 99
#define MAVLINK_MSG_ID_12002_CRC 99



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DC_MOTOR_CONTROL { \
    12002, \
    "DC_MOTOR_CONTROL", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_dc_motor_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_dc_motor_control_t, target_component) }, \
         { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_dc_motor_control_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_dc_motor_control_t, control_mode) }, \
         { "target_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dc_motor_control_t, target_value) }, \
         { "speed_limit_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dc_motor_control_t, speed_limit_rad_s) }, \
         { "acceleration_limit_rad_s2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dc_motor_control_t, acceleration_limit_rad_s2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DC_MOTOR_CONTROL { \
    "DC_MOTOR_CONTROL", \
    7, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_dc_motor_control_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_dc_motor_control_t, target_component) }, \
         { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_dc_motor_control_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_dc_motor_control_t, control_mode) }, \
         { "target_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dc_motor_control_t, target_value) }, \
         { "speed_limit_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dc_motor_control_t, speed_limit_rad_s) }, \
         { "acceleration_limit_rad_s2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dc_motor_control_t, acceleration_limit_rad_s2) }, \
         } \
}
#endif

/**
 * @brief Pack a dc_motor_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=PWM/Open Loop, 1=Speed Control, 2=Position Control, 3=Disabled)
 * @param target_value  Target value (PWM duty -1.0 to 1.0, speed in rad/s, or position in rad)
 * @param speed_limit_rad_s  Speed limit in rad/s (0 = use motor default limit)
 * @param acceleration_limit_rad_s2  Acceleration limit in rad/s^2 (0 = use motor default limit)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dc_motor_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float target_value, float speed_limit_rad_s, float acceleration_limit_rad_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_float(buf, 4, speed_limit_rad_s);
    _mav_put_float(buf, 8, acceleration_limit_rad_s2);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, motor_id);
    _mav_put_uint8_t(buf, 15, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#else
    mavlink_dc_motor_control_t packet;
    packet.target_value = target_value;
    packet.speed_limit_rad_s = speed_limit_rad_s;
    packet.acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DC_MOTOR_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
}

/**
 * @brief Pack a dc_motor_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=PWM/Open Loop, 1=Speed Control, 2=Position Control, 3=Disabled)
 * @param target_value  Target value (PWM duty -1.0 to 1.0, speed in rad/s, or position in rad)
 * @param speed_limit_rad_s  Speed limit in rad/s (0 = use motor default limit)
 * @param acceleration_limit_rad_s2  Acceleration limit in rad/s^2 (0 = use motor default limit)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dc_motor_control_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float target_value, float speed_limit_rad_s, float acceleration_limit_rad_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_float(buf, 4, speed_limit_rad_s);
    _mav_put_float(buf, 8, acceleration_limit_rad_s2);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, motor_id);
    _mav_put_uint8_t(buf, 15, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#else
    mavlink_dc_motor_control_t packet;
    packet.target_value = target_value;
    packet.speed_limit_rad_s = speed_limit_rad_s;
    packet.acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DC_MOTOR_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a dc_motor_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=PWM/Open Loop, 1=Speed Control, 2=Position Control, 3=Disabled)
 * @param target_value  Target value (PWM duty -1.0 to 1.0, speed in rad/s, or position in rad)
 * @param speed_limit_rad_s  Speed limit in rad/s (0 = use motor default limit)
 * @param acceleration_limit_rad_s2  Acceleration limit in rad/s^2 (0 = use motor default limit)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dc_motor_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint8_t motor_id,uint8_t control_mode,float target_value,float speed_limit_rad_s,float acceleration_limit_rad_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_float(buf, 4, speed_limit_rad_s);
    _mav_put_float(buf, 8, acceleration_limit_rad_s2);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, motor_id);
    _mav_put_uint8_t(buf, 15, control_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#else
    mavlink_dc_motor_control_t packet;
    packet.target_value = target_value;
    packet.speed_limit_rad_s = speed_limit_rad_s;
    packet.acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DC_MOTOR_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
}

/**
 * @brief Encode a dc_motor_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dc_motor_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dc_motor_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dc_motor_control_t* dc_motor_control)
{
    return mavlink_msg_dc_motor_control_pack(system_id, component_id, msg, dc_motor_control->target_system, dc_motor_control->target_component, dc_motor_control->motor_id, dc_motor_control->control_mode, dc_motor_control->target_value, dc_motor_control->speed_limit_rad_s, dc_motor_control->acceleration_limit_rad_s2);
}

/**
 * @brief Encode a dc_motor_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dc_motor_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dc_motor_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dc_motor_control_t* dc_motor_control)
{
    return mavlink_msg_dc_motor_control_pack_chan(system_id, component_id, chan, msg, dc_motor_control->target_system, dc_motor_control->target_component, dc_motor_control->motor_id, dc_motor_control->control_mode, dc_motor_control->target_value, dc_motor_control->speed_limit_rad_s, dc_motor_control->acceleration_limit_rad_s2);
}

/**
 * @brief Encode a dc_motor_control struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param dc_motor_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dc_motor_control_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_dc_motor_control_t* dc_motor_control)
{
    return mavlink_msg_dc_motor_control_pack_status(system_id, component_id, _status, msg,  dc_motor_control->target_system, dc_motor_control->target_component, dc_motor_control->motor_id, dc_motor_control->control_mode, dc_motor_control->target_value, dc_motor_control->speed_limit_rad_s, dc_motor_control->acceleration_limit_rad_s2);
}

/**
 * @brief Send a dc_motor_control message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Control mode (0=PWM/Open Loop, 1=Speed Control, 2=Position Control, 3=Disabled)
 * @param target_value  Target value (PWM duty -1.0 to 1.0, speed in rad/s, or position in rad)
 * @param speed_limit_rad_s  Speed limit in rad/s (0 = use motor default limit)
 * @param acceleration_limit_rad_s2  Acceleration limit in rad/s^2 (0 = use motor default limit)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dc_motor_control_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float target_value, float speed_limit_rad_s, float acceleration_limit_rad_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN];
    _mav_put_float(buf, 0, target_value);
    _mav_put_float(buf, 4, speed_limit_rad_s);
    _mav_put_float(buf, 8, acceleration_limit_rad_s2);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, motor_id);
    _mav_put_uint8_t(buf, 15, control_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_CONTROL, buf, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
#else
    mavlink_dc_motor_control_t packet;
    packet.target_value = target_value;
    packet.speed_limit_rad_s = speed_limit_rad_s;
    packet.acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
#endif
}

/**
 * @brief Send a dc_motor_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dc_motor_control_send_struct(mavlink_channel_t chan, const mavlink_dc_motor_control_t* dc_motor_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dc_motor_control_send(chan, dc_motor_control->target_system, dc_motor_control->target_component, dc_motor_control->motor_id, dc_motor_control->control_mode, dc_motor_control->target_value, dc_motor_control->speed_limit_rad_s, dc_motor_control->acceleration_limit_rad_s2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_CONTROL, (const char *)dc_motor_control, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dc_motor_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t motor_id, uint8_t control_mode, float target_value, float speed_limit_rad_s, float acceleration_limit_rad_s2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, target_value);
    _mav_put_float(buf, 4, speed_limit_rad_s);
    _mav_put_float(buf, 8, acceleration_limit_rad_s2);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_uint8_t(buf, 14, motor_id);
    _mav_put_uint8_t(buf, 15, control_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_CONTROL, buf, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
#else
    mavlink_dc_motor_control_t *packet = (mavlink_dc_motor_control_t *)msgbuf;
    packet->target_value = target_value;
    packet->speed_limit_rad_s = speed_limit_rad_s;
    packet->acceleration_limit_rad_s2 = acceleration_limit_rad_s2;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->motor_id = motor_id;
    packet->control_mode = control_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_CONTROL, (const char *)packet, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE DC_MOTOR_CONTROL UNPACKING


/**
 * @brief Get field target_system from dc_motor_control message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_dc_motor_control_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from dc_motor_control message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_dc_motor_control_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field motor_id from dc_motor_control message
 *
 * @return  Motor ID (1-255)
 */
static inline uint8_t mavlink_msg_dc_motor_control_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field control_mode from dc_motor_control message
 *
 * @return  Control mode (0=PWM/Open Loop, 1=Speed Control, 2=Position Control, 3=Disabled)
 */
static inline uint8_t mavlink_msg_dc_motor_control_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field target_value from dc_motor_control message
 *
 * @return  Target value (PWM duty -1.0 to 1.0, speed in rad/s, or position in rad)
 */
static inline float mavlink_msg_dc_motor_control_get_target_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field speed_limit_rad_s from dc_motor_control message
 *
 * @return  Speed limit in rad/s (0 = use motor default limit)
 */
static inline float mavlink_msg_dc_motor_control_get_speed_limit_rad_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field acceleration_limit_rad_s2 from dc_motor_control message
 *
 * @return  Acceleration limit in rad/s^2 (0 = use motor default limit)
 */
static inline float mavlink_msg_dc_motor_control_get_acceleration_limit_rad_s2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a dc_motor_control message into a struct
 *
 * @param msg The message to decode
 * @param dc_motor_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_dc_motor_control_decode(const mavlink_message_t* msg, mavlink_dc_motor_control_t* dc_motor_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dc_motor_control->target_value = mavlink_msg_dc_motor_control_get_target_value(msg);
    dc_motor_control->speed_limit_rad_s = mavlink_msg_dc_motor_control_get_speed_limit_rad_s(msg);
    dc_motor_control->acceleration_limit_rad_s2 = mavlink_msg_dc_motor_control_get_acceleration_limit_rad_s2(msg);
    dc_motor_control->target_system = mavlink_msg_dc_motor_control_get_target_system(msg);
    dc_motor_control->target_component = mavlink_msg_dc_motor_control_get_target_component(msg);
    dc_motor_control->motor_id = mavlink_msg_dc_motor_control_get_motor_id(msg);
    dc_motor_control->control_mode = mavlink_msg_dc_motor_control_get_control_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN;
        memset(dc_motor_control, 0, MAVLINK_MSG_ID_DC_MOTOR_CONTROL_LEN);
    memcpy(dc_motor_control, _MAV_PAYLOAD(msg), len);
#endif
}
