#pragma once
// MESSAGE DC_MOTOR_STATUS PACKING

#define MAVLINK_MSG_ID_DC_MOTOR_STATUS 12003


typedef struct __mavlink_dc_motor_status_t {
 float position_rad; /*<  Current position in radians*/
 float speed_rad_s; /*<  Current speed in rad/s*/
 float duty_cycle; /*<  Current PWM duty cycle (-1.0 to 1.0)*/
 float position_error_rad; /*<  Position error in radians (only valid in position mode)*/
 float target_value; /*<  Current target value (depends on control mode)*/
 uint32_t timestamp_ms; /*<  Timestamp in milliseconds*/
 uint8_t motor_id; /*<  Motor ID (1-255)*/
 uint8_t control_mode; /*<  Current control mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)*/
 uint8_t status; /*<  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=LIMIT_REACHED)*/
} mavlink_dc_motor_status_t;

#define MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN 27
#define MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN 27
#define MAVLINK_MSG_ID_12003_LEN 27
#define MAVLINK_MSG_ID_12003_MIN_LEN 27

#define MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC 54
#define MAVLINK_MSG_ID_12003_CRC 54



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DC_MOTOR_STATUS { \
    12003, \
    "DC_MOTOR_STATUS", \
    9, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_dc_motor_status_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_dc_motor_status_t, control_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_dc_motor_status_t, status) }, \
         { "position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dc_motor_status_t, position_rad) }, \
         { "speed_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dc_motor_status_t, speed_rad_s) }, \
         { "duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dc_motor_status_t, duty_cycle) }, \
         { "position_error_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_dc_motor_status_t, position_error_rad) }, \
         { "target_value", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_dc_motor_status_t, target_value) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_dc_motor_status_t, timestamp_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DC_MOTOR_STATUS { \
    "DC_MOTOR_STATUS", \
    9, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_dc_motor_status_t, motor_id) }, \
         { "control_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_dc_motor_status_t, control_mode) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_dc_motor_status_t, status) }, \
         { "position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dc_motor_status_t, position_rad) }, \
         { "speed_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dc_motor_status_t, speed_rad_s) }, \
         { "duty_cycle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dc_motor_status_t, duty_cycle) }, \
         { "position_error_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_dc_motor_status_t, position_error_rad) }, \
         { "target_value", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_dc_motor_status_t, target_value) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_dc_motor_status_t, timestamp_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a dc_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=LIMIT_REACHED)
 * @param position_rad  Current position in radians
 * @param speed_rad_s  Current speed in rad/s
 * @param duty_cycle  Current PWM duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (only valid in position mode)
 * @param target_value  Current target value (depends on control mode)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dc_motor_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, uint8_t status, float position_rad, float speed_rad_s, float duty_cycle, float position_error_rad, float target_value, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, position_rad);
    _mav_put_float(buf, 4, speed_rad_s);
    _mav_put_float(buf, 8, duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_float(buf, 16, target_value);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, motor_id);
    _mav_put_uint8_t(buf, 25, control_mode);
    _mav_put_uint8_t(buf, 26, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#else
    mavlink_dc_motor_status_t packet;
    packet.position_rad = position_rad;
    packet.speed_rad_s = speed_rad_s;
    packet.duty_cycle = duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.target_value = target_value;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DC_MOTOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
}

/**
 * @brief Pack a dc_motor_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=LIMIT_REACHED)
 * @param position_rad  Current position in radians
 * @param speed_rad_s  Current speed in rad/s
 * @param duty_cycle  Current PWM duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (only valid in position mode)
 * @param target_value  Current target value (depends on control mode)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dc_motor_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint8_t control_mode, uint8_t status, float position_rad, float speed_rad_s, float duty_cycle, float position_error_rad, float target_value, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, position_rad);
    _mav_put_float(buf, 4, speed_rad_s);
    _mav_put_float(buf, 8, duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_float(buf, 16, target_value);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, motor_id);
    _mav_put_uint8_t(buf, 25, control_mode);
    _mav_put_uint8_t(buf, 26, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#else
    mavlink_dc_motor_status_t packet;
    packet.position_rad = position_rad;
    packet.speed_rad_s = speed_rad_s;
    packet.duty_cycle = duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.target_value = target_value;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DC_MOTOR_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#endif
}

/**
 * @brief Pack a dc_motor_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=LIMIT_REACHED)
 * @param position_rad  Current position in radians
 * @param speed_rad_s  Current speed in rad/s
 * @param duty_cycle  Current PWM duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (only valid in position mode)
 * @param target_value  Current target value (depends on control mode)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dc_motor_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint8_t control_mode,uint8_t status,float position_rad,float speed_rad_s,float duty_cycle,float position_error_rad,float target_value,uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, position_rad);
    _mav_put_float(buf, 4, speed_rad_s);
    _mav_put_float(buf, 8, duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_float(buf, 16, target_value);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, motor_id);
    _mav_put_uint8_t(buf, 25, control_mode);
    _mav_put_uint8_t(buf, 26, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#else
    mavlink_dc_motor_status_t packet;
    packet.position_rad = position_rad;
    packet.speed_rad_s = speed_rad_s;
    packet.duty_cycle = duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.target_value = target_value;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DC_MOTOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
}

/**
 * @brief Encode a dc_motor_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dc_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dc_motor_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dc_motor_status_t* dc_motor_status)
{
    return mavlink_msg_dc_motor_status_pack(system_id, component_id, msg, dc_motor_status->motor_id, dc_motor_status->control_mode, dc_motor_status->status, dc_motor_status->position_rad, dc_motor_status->speed_rad_s, dc_motor_status->duty_cycle, dc_motor_status->position_error_rad, dc_motor_status->target_value, dc_motor_status->timestamp_ms);
}

/**
 * @brief Encode a dc_motor_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dc_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dc_motor_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dc_motor_status_t* dc_motor_status)
{
    return mavlink_msg_dc_motor_status_pack_chan(system_id, component_id, chan, msg, dc_motor_status->motor_id, dc_motor_status->control_mode, dc_motor_status->status, dc_motor_status->position_rad, dc_motor_status->speed_rad_s, dc_motor_status->duty_cycle, dc_motor_status->position_error_rad, dc_motor_status->target_value, dc_motor_status->timestamp_ms);
}

/**
 * @brief Encode a dc_motor_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param dc_motor_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dc_motor_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_dc_motor_status_t* dc_motor_status)
{
    return mavlink_msg_dc_motor_status_pack_status(system_id, component_id, _status, msg,  dc_motor_status->motor_id, dc_motor_status->control_mode, dc_motor_status->status, dc_motor_status->position_rad, dc_motor_status->speed_rad_s, dc_motor_status->duty_cycle, dc_motor_status->position_error_rad, dc_motor_status->target_value, dc_motor_status->timestamp_ms);
}

/**
 * @brief Send a dc_motor_status message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (1-255)
 * @param control_mode  Current control mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)
 * @param status  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=LIMIT_REACHED)
 * @param position_rad  Current position in radians
 * @param speed_rad_s  Current speed in rad/s
 * @param duty_cycle  Current PWM duty cycle (-1.0 to 1.0)
 * @param position_error_rad  Position error in radians (only valid in position mode)
 * @param target_value  Current target value (depends on control mode)
 * @param timestamp_ms  Timestamp in milliseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dc_motor_status_send(mavlink_channel_t chan, uint8_t motor_id, uint8_t control_mode, uint8_t status, float position_rad, float speed_rad_s, float duty_cycle, float position_error_rad, float target_value, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN];
    _mav_put_float(buf, 0, position_rad);
    _mav_put_float(buf, 4, speed_rad_s);
    _mav_put_float(buf, 8, duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_float(buf, 16, target_value);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, motor_id);
    _mav_put_uint8_t(buf, 25, control_mode);
    _mav_put_uint8_t(buf, 26, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_STATUS, buf, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
#else
    mavlink_dc_motor_status_t packet;
    packet.position_rad = position_rad;
    packet.speed_rad_s = speed_rad_s;
    packet.duty_cycle = duty_cycle;
    packet.position_error_rad = position_error_rad;
    packet.target_value = target_value;
    packet.timestamp_ms = timestamp_ms;
    packet.motor_id = motor_id;
    packet.control_mode = control_mode;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a dc_motor_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dc_motor_status_send_struct(mavlink_channel_t chan, const mavlink_dc_motor_status_t* dc_motor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dc_motor_status_send(chan, dc_motor_status->motor_id, dc_motor_status->control_mode, dc_motor_status->status, dc_motor_status->position_rad, dc_motor_status->speed_rad_s, dc_motor_status->duty_cycle, dc_motor_status->position_error_rad, dc_motor_status->target_value, dc_motor_status->timestamp_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_STATUS, (const char *)dc_motor_status, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dc_motor_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint8_t control_mode, uint8_t status, float position_rad, float speed_rad_s, float duty_cycle, float position_error_rad, float target_value, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, position_rad);
    _mav_put_float(buf, 4, speed_rad_s);
    _mav_put_float(buf, 8, duty_cycle);
    _mav_put_float(buf, 12, position_error_rad);
    _mav_put_float(buf, 16, target_value);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, motor_id);
    _mav_put_uint8_t(buf, 25, control_mode);
    _mav_put_uint8_t(buf, 26, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_STATUS, buf, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
#else
    mavlink_dc_motor_status_t *packet = (mavlink_dc_motor_status_t *)msgbuf;
    packet->position_rad = position_rad;
    packet->speed_rad_s = speed_rad_s;
    packet->duty_cycle = duty_cycle;
    packet->position_error_rad = position_error_rad;
    packet->target_value = target_value;
    packet->timestamp_ms = timestamp_ms;
    packet->motor_id = motor_id;
    packet->control_mode = control_mode;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DC_MOTOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN, MAVLINK_MSG_ID_DC_MOTOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE DC_MOTOR_STATUS UNPACKING


/**
 * @brief Get field motor_id from dc_motor_status message
 *
 * @return  Motor ID (1-255)
 */
static inline uint8_t mavlink_msg_dc_motor_status_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field control_mode from dc_motor_status message
 *
 * @return  Current control mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)
 */
static inline uint8_t mavlink_msg_dc_motor_status_get_control_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field status from dc_motor_status message
 *
 * @return  Motor status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=LIMIT_REACHED)
 */
static inline uint8_t mavlink_msg_dc_motor_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field position_rad from dc_motor_status message
 *
 * @return  Current position in radians
 */
static inline float mavlink_msg_dc_motor_status_get_position_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field speed_rad_s from dc_motor_status message
 *
 * @return  Current speed in rad/s
 */
static inline float mavlink_msg_dc_motor_status_get_speed_rad_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field duty_cycle from dc_motor_status message
 *
 * @return  Current PWM duty cycle (-1.0 to 1.0)
 */
static inline float mavlink_msg_dc_motor_status_get_duty_cycle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field position_error_rad from dc_motor_status message
 *
 * @return  Position error in radians (only valid in position mode)
 */
static inline float mavlink_msg_dc_motor_status_get_position_error_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field target_value from dc_motor_status message
 *
 * @return  Current target value (depends on control mode)
 */
static inline float mavlink_msg_dc_motor_status_get_target_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field timestamp_ms from dc_motor_status message
 *
 * @return  Timestamp in milliseconds
 */
static inline uint32_t mavlink_msg_dc_motor_status_get_timestamp_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a dc_motor_status message into a struct
 *
 * @param msg The message to decode
 * @param dc_motor_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_dc_motor_status_decode(const mavlink_message_t* msg, mavlink_dc_motor_status_t* dc_motor_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dc_motor_status->position_rad = mavlink_msg_dc_motor_status_get_position_rad(msg);
    dc_motor_status->speed_rad_s = mavlink_msg_dc_motor_status_get_speed_rad_s(msg);
    dc_motor_status->duty_cycle = mavlink_msg_dc_motor_status_get_duty_cycle(msg);
    dc_motor_status->position_error_rad = mavlink_msg_dc_motor_status_get_position_error_rad(msg);
    dc_motor_status->target_value = mavlink_msg_dc_motor_status_get_target_value(msg);
    dc_motor_status->timestamp_ms = mavlink_msg_dc_motor_status_get_timestamp_ms(msg);
    dc_motor_status->motor_id = mavlink_msg_dc_motor_status_get_motor_id(msg);
    dc_motor_status->control_mode = mavlink_msg_dc_motor_status_get_control_mode(msg);
    dc_motor_status->status = mavlink_msg_dc_motor_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN;
        memset(dc_motor_status, 0, MAVLINK_MSG_ID_DC_MOTOR_STATUS_LEN);
    memcpy(dc_motor_status, _MAV_PAYLOAD(msg), len);
#endif
}
