#pragma once
// MESSAGE ENCODER_STATUS PACKING

#define MAVLINK_MSG_ID_ENCODER_STATUS 12006


typedef struct __mavlink_encoder_status_t {
 int32_t raw_count; /*<  Raw encoder count (signed 32-bit)*/
 float position_rad; /*<  Current position in radians*/
 float velocity_rad_s; /*<  Current velocity in rad/s*/
 uint32_t resolution; /*<  Encoder resolution (counts per revolution)*/
 float gear_ratio; /*<  Gear ratio (output/input)*/
 uint32_t timestamp_ms; /*<  Timestamp in milliseconds*/
 uint8_t encoder_id; /*<  Encoder ID (50-59 for standalone encoders)*/
 uint8_t encoder_type; /*<  Encoder type (0=incremental, 1=absolute, 2=magnetic)*/
 uint8_t status; /*<  Encoder status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=OVERFLOW)*/
} mavlink_encoder_status_t;

#define MAVLINK_MSG_ID_ENCODER_STATUS_LEN 27
#define MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN 27
#define MAVLINK_MSG_ID_12006_LEN 27
#define MAVLINK_MSG_ID_12006_MIN_LEN 27

#define MAVLINK_MSG_ID_ENCODER_STATUS_CRC 208
#define MAVLINK_MSG_ID_12006_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ENCODER_STATUS { \
    12006, \
    "ENCODER_STATUS", \
    9, \
    {  { "encoder_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_encoder_status_t, encoder_id) }, \
         { "encoder_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_encoder_status_t, encoder_type) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_encoder_status_t, status) }, \
         { "raw_count", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_encoder_status_t, raw_count) }, \
         { "position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_encoder_status_t, position_rad) }, \
         { "velocity_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_encoder_status_t, velocity_rad_s) }, \
         { "resolution", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_encoder_status_t, resolution) }, \
         { "gear_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_encoder_status_t, gear_ratio) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_encoder_status_t, timestamp_ms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ENCODER_STATUS { \
    "ENCODER_STATUS", \
    9, \
    {  { "encoder_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_encoder_status_t, encoder_id) }, \
         { "encoder_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_encoder_status_t, encoder_type) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_encoder_status_t, status) }, \
         { "raw_count", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_encoder_status_t, raw_count) }, \
         { "position_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_encoder_status_t, position_rad) }, \
         { "velocity_rad_s", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_encoder_status_t, velocity_rad_s) }, \
         { "resolution", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_encoder_status_t, resolution) }, \
         { "gear_ratio", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_encoder_status_t, gear_ratio) }, \
         { "timestamp_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_encoder_status_t, timestamp_ms) }, \
         } \
}
#endif

/**
 * @brief Pack a encoder_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder_id  Encoder ID (50-59 for standalone encoders)
 * @param encoder_type  Encoder type (0=incremental, 1=absolute, 2=magnetic)
 * @param status  Encoder status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=OVERFLOW)
 * @param raw_count  Raw encoder count (signed 32-bit)
 * @param position_rad  Current position in radians
 * @param velocity_rad_s  Current velocity in rad/s
 * @param resolution  Encoder resolution (counts per revolution)
 * @param gear_ratio  Gear ratio (output/input)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t encoder_id, uint8_t encoder_type, uint8_t status, int32_t raw_count, float position_rad, float velocity_rad_s, uint32_t resolution, float gear_ratio, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_STATUS_LEN];
    _mav_put_int32_t(buf, 0, raw_count);
    _mav_put_float(buf, 4, position_rad);
    _mav_put_float(buf, 8, velocity_rad_s);
    _mav_put_uint32_t(buf, 12, resolution);
    _mav_put_float(buf, 16, gear_ratio);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, encoder_id);
    _mav_put_uint8_t(buf, 25, encoder_type);
    _mav_put_uint8_t(buf, 26, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#else
    mavlink_encoder_status_t packet;
    packet.raw_count = raw_count;
    packet.position_rad = position_rad;
    packet.velocity_rad_s = velocity_rad_s;
    packet.resolution = resolution;
    packet.gear_ratio = gear_ratio;
    packet.timestamp_ms = timestamp_ms;
    packet.encoder_id = encoder_id;
    packet.encoder_type = encoder_type;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENCODER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
}

/**
 * @brief Pack a encoder_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param encoder_id  Encoder ID (50-59 for standalone encoders)
 * @param encoder_type  Encoder type (0=incremental, 1=absolute, 2=magnetic)
 * @param status  Encoder status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=OVERFLOW)
 * @param raw_count  Raw encoder count (signed 32-bit)
 * @param position_rad  Current position in radians
 * @param velocity_rad_s  Current velocity in rad/s
 * @param resolution  Encoder resolution (counts per revolution)
 * @param gear_ratio  Gear ratio (output/input)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t encoder_id, uint8_t encoder_type, uint8_t status, int32_t raw_count, float position_rad, float velocity_rad_s, uint32_t resolution, float gear_ratio, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_STATUS_LEN];
    _mav_put_int32_t(buf, 0, raw_count);
    _mav_put_float(buf, 4, position_rad);
    _mav_put_float(buf, 8, velocity_rad_s);
    _mav_put_uint32_t(buf, 12, resolution);
    _mav_put_float(buf, 16, gear_ratio);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, encoder_id);
    _mav_put_uint8_t(buf, 25, encoder_type);
    _mav_put_uint8_t(buf, 26, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#else
    mavlink_encoder_status_t packet;
    packet.raw_count = raw_count;
    packet.position_rad = position_rad;
    packet.velocity_rad_s = velocity_rad_s;
    packet.resolution = resolution;
    packet.gear_ratio = gear_ratio;
    packet.timestamp_ms = timestamp_ms;
    packet.encoder_id = encoder_id;
    packet.encoder_type = encoder_type;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENCODER_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#endif
}

/**
 * @brief Pack a encoder_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder_id  Encoder ID (50-59 for standalone encoders)
 * @param encoder_type  Encoder type (0=incremental, 1=absolute, 2=magnetic)
 * @param status  Encoder status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=OVERFLOW)
 * @param raw_count  Raw encoder count (signed 32-bit)
 * @param position_rad  Current position in radians
 * @param velocity_rad_s  Current velocity in rad/s
 * @param resolution  Encoder resolution (counts per revolution)
 * @param gear_ratio  Gear ratio (output/input)
 * @param timestamp_ms  Timestamp in milliseconds
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_encoder_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t encoder_id,uint8_t encoder_type,uint8_t status,int32_t raw_count,float position_rad,float velocity_rad_s,uint32_t resolution,float gear_ratio,uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_STATUS_LEN];
    _mav_put_int32_t(buf, 0, raw_count);
    _mav_put_float(buf, 4, position_rad);
    _mav_put_float(buf, 8, velocity_rad_s);
    _mav_put_uint32_t(buf, 12, resolution);
    _mav_put_float(buf, 16, gear_ratio);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, encoder_id);
    _mav_put_uint8_t(buf, 25, encoder_type);
    _mav_put_uint8_t(buf, 26, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#else
    mavlink_encoder_status_t packet;
    packet.raw_count = raw_count;
    packet.position_rad = position_rad;
    packet.velocity_rad_s = velocity_rad_s;
    packet.resolution = resolution;
    packet.gear_ratio = gear_ratio;
    packet.timestamp_ms = timestamp_ms;
    packet.encoder_id = encoder_id;
    packet.encoder_type = encoder_type;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENCODER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
}

/**
 * @brief Encode a encoder_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param encoder_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_encoder_status_t* encoder_status)
{
    return mavlink_msg_encoder_status_pack(system_id, component_id, msg, encoder_status->encoder_id, encoder_status->encoder_type, encoder_status->status, encoder_status->raw_count, encoder_status->position_rad, encoder_status->velocity_rad_s, encoder_status->resolution, encoder_status->gear_ratio, encoder_status->timestamp_ms);
}

/**
 * @brief Encode a encoder_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param encoder_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_encoder_status_t* encoder_status)
{
    return mavlink_msg_encoder_status_pack_chan(system_id, component_id, chan, msg, encoder_status->encoder_id, encoder_status->encoder_type, encoder_status->status, encoder_status->raw_count, encoder_status->position_rad, encoder_status->velocity_rad_s, encoder_status->resolution, encoder_status->gear_ratio, encoder_status->timestamp_ms);
}

/**
 * @brief Encode a encoder_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param encoder_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_encoder_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_encoder_status_t* encoder_status)
{
    return mavlink_msg_encoder_status_pack_status(system_id, component_id, _status, msg,  encoder_status->encoder_id, encoder_status->encoder_type, encoder_status->status, encoder_status->raw_count, encoder_status->position_rad, encoder_status->velocity_rad_s, encoder_status->resolution, encoder_status->gear_ratio, encoder_status->timestamp_ms);
}

/**
 * @brief Send a encoder_status message
 * @param chan MAVLink channel to send the message
 *
 * @param encoder_id  Encoder ID (50-59 for standalone encoders)
 * @param encoder_type  Encoder type (0=incremental, 1=absolute, 2=magnetic)
 * @param status  Encoder status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=OVERFLOW)
 * @param raw_count  Raw encoder count (signed 32-bit)
 * @param position_rad  Current position in radians
 * @param velocity_rad_s  Current velocity in rad/s
 * @param resolution  Encoder resolution (counts per revolution)
 * @param gear_ratio  Gear ratio (output/input)
 * @param timestamp_ms  Timestamp in milliseconds
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_encoder_status_send(mavlink_channel_t chan, uint8_t encoder_id, uint8_t encoder_type, uint8_t status, int32_t raw_count, float position_rad, float velocity_rad_s, uint32_t resolution, float gear_ratio, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENCODER_STATUS_LEN];
    _mav_put_int32_t(buf, 0, raw_count);
    _mav_put_float(buf, 4, position_rad);
    _mav_put_float(buf, 8, velocity_rad_s);
    _mav_put_uint32_t(buf, 12, resolution);
    _mav_put_float(buf, 16, gear_ratio);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, encoder_id);
    _mav_put_uint8_t(buf, 25, encoder_type);
    _mav_put_uint8_t(buf, 26, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_STATUS, buf, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
#else
    mavlink_encoder_status_t packet;
    packet.raw_count = raw_count;
    packet.position_rad = position_rad;
    packet.velocity_rad_s = velocity_rad_s;
    packet.resolution = resolution;
    packet.gear_ratio = gear_ratio;
    packet.timestamp_ms = timestamp_ms;
    packet.encoder_id = encoder_id;
    packet.encoder_type = encoder_type;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
#endif
}

/**
 * @brief Send a encoder_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_encoder_status_send_struct(mavlink_channel_t chan, const mavlink_encoder_status_t* encoder_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_encoder_status_send(chan, encoder_status->encoder_id, encoder_status->encoder_type, encoder_status->status, encoder_status->raw_count, encoder_status->position_rad, encoder_status->velocity_rad_s, encoder_status->resolution, encoder_status->gear_ratio, encoder_status->timestamp_ms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_STATUS, (const char *)encoder_status, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ENCODER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_encoder_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t encoder_id, uint8_t encoder_type, uint8_t status, int32_t raw_count, float position_rad, float velocity_rad_s, uint32_t resolution, float gear_ratio, uint32_t timestamp_ms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, raw_count);
    _mav_put_float(buf, 4, position_rad);
    _mav_put_float(buf, 8, velocity_rad_s);
    _mav_put_uint32_t(buf, 12, resolution);
    _mav_put_float(buf, 16, gear_ratio);
    _mav_put_uint32_t(buf, 20, timestamp_ms);
    _mav_put_uint8_t(buf, 24, encoder_id);
    _mav_put_uint8_t(buf, 25, encoder_type);
    _mav_put_uint8_t(buf, 26, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_STATUS, buf, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
#else
    mavlink_encoder_status_t *packet = (mavlink_encoder_status_t *)msgbuf;
    packet->raw_count = raw_count;
    packet->position_rad = position_rad;
    packet->velocity_rad_s = velocity_rad_s;
    packet->resolution = resolution;
    packet->gear_ratio = gear_ratio;
    packet->timestamp_ms = timestamp_ms;
    packet->encoder_id = encoder_id;
    packet->encoder_type = encoder_type;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENCODER_STATUS, (const char *)packet, MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_LEN, MAVLINK_MSG_ID_ENCODER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ENCODER_STATUS UNPACKING


/**
 * @brief Get field encoder_id from encoder_status message
 *
 * @return  Encoder ID (50-59 for standalone encoders)
 */
static inline uint8_t mavlink_msg_encoder_status_get_encoder_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field encoder_type from encoder_status message
 *
 * @return  Encoder type (0=incremental, 1=absolute, 2=magnetic)
 */
static inline uint8_t mavlink_msg_encoder_status_get_encoder_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field status from encoder_status message
 *
 * @return  Encoder status (0=OK, 1=ERROR, 2=TIMEOUT, 3=NOT_INITIALIZED, 4=OVERFLOW)
 */
static inline uint8_t mavlink_msg_encoder_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field raw_count from encoder_status message
 *
 * @return  Raw encoder count (signed 32-bit)
 */
static inline int32_t mavlink_msg_encoder_status_get_raw_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field position_rad from encoder_status message
 *
 * @return  Current position in radians
 */
static inline float mavlink_msg_encoder_status_get_position_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field velocity_rad_s from encoder_status message
 *
 * @return  Current velocity in rad/s
 */
static inline float mavlink_msg_encoder_status_get_velocity_rad_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field resolution from encoder_status message
 *
 * @return  Encoder resolution (counts per revolution)
 */
static inline uint32_t mavlink_msg_encoder_status_get_resolution(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field gear_ratio from encoder_status message
 *
 * @return  Gear ratio (output/input)
 */
static inline float mavlink_msg_encoder_status_get_gear_ratio(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field timestamp_ms from encoder_status message
 *
 * @return  Timestamp in milliseconds
 */
static inline uint32_t mavlink_msg_encoder_status_get_timestamp_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Decode a encoder_status message into a struct
 *
 * @param msg The message to decode
 * @param encoder_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_encoder_status_decode(const mavlink_message_t* msg, mavlink_encoder_status_t* encoder_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    encoder_status->raw_count = mavlink_msg_encoder_status_get_raw_count(msg);
    encoder_status->position_rad = mavlink_msg_encoder_status_get_position_rad(msg);
    encoder_status->velocity_rad_s = mavlink_msg_encoder_status_get_velocity_rad_s(msg);
    encoder_status->resolution = mavlink_msg_encoder_status_get_resolution(msg);
    encoder_status->gear_ratio = mavlink_msg_encoder_status_get_gear_ratio(msg);
    encoder_status->timestamp_ms = mavlink_msg_encoder_status_get_timestamp_ms(msg);
    encoder_status->encoder_id = mavlink_msg_encoder_status_get_encoder_id(msg);
    encoder_status->encoder_type = mavlink_msg_encoder_status_get_encoder_type(msg);
    encoder_status->status = mavlink_msg_encoder_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ENCODER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ENCODER_STATUS_LEN;
        memset(encoder_status, 0, MAVLINK_MSG_ID_ENCODER_STATUS_LEN);
    memcpy(encoder_status, _MAV_PAYLOAD(msg), len);
#endif
}
