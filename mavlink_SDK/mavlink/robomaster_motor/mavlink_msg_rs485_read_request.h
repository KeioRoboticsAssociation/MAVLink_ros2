#pragma once
// MESSAGE RS485_READ_REQUEST PACKING

#define MAVLINK_MSG_ID_RS485_READ_REQUEST 12007


typedef struct __mavlink_rs485_read_request_t {
 uint16_t address; /*<  Control table address to read*/
 uint8_t motor_id; /*<  Motor ID (30-49 for RS485 motors)*/
 uint8_t length; /*<  Number of bytes to read (1-64)*/
} mavlink_rs485_read_request_t;

#define MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN 4
#define MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN 4
#define MAVLINK_MSG_ID_12007_LEN 4
#define MAVLINK_MSG_ID_12007_MIN_LEN 4

#define MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC 202
#define MAVLINK_MSG_ID_12007_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RS485_READ_REQUEST { \
    12007, \
    "RS485_READ_REQUEST", \
    3, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rs485_read_request_t, motor_id) }, \
         { "address", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rs485_read_request_t, address) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_rs485_read_request_t, length) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RS485_READ_REQUEST { \
    "RS485_READ_REQUEST", \
    3, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rs485_read_request_t, motor_id) }, \
         { "address", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rs485_read_request_t, address) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_rs485_read_request_t, length) }, \
         } \
}
#endif

/**
 * @brief Pack a rs485_read_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address to read
 * @param length  Number of bytes to read (1-64)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_read_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint16_t address, uint8_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#else
    mavlink_rs485_read_request_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_READ_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
}

/**
 * @brief Pack a rs485_read_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address to read
 * @param length  Number of bytes to read (1-64)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_read_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint16_t address, uint8_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#else
    mavlink_rs485_read_request_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_READ_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a rs485_read_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address to read
 * @param length  Number of bytes to read (1-64)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_read_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint16_t address,uint8_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#else
    mavlink_rs485_read_request_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_READ_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
}

/**
 * @brief Encode a rs485_read_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rs485_read_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_read_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rs485_read_request_t* rs485_read_request)
{
    return mavlink_msg_rs485_read_request_pack(system_id, component_id, msg, rs485_read_request->motor_id, rs485_read_request->address, rs485_read_request->length);
}

/**
 * @brief Encode a rs485_read_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rs485_read_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_read_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rs485_read_request_t* rs485_read_request)
{
    return mavlink_msg_rs485_read_request_pack_chan(system_id, component_id, chan, msg, rs485_read_request->motor_id, rs485_read_request->address, rs485_read_request->length);
}

/**
 * @brief Encode a rs485_read_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rs485_read_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_read_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rs485_read_request_t* rs485_read_request)
{
    return mavlink_msg_rs485_read_request_pack_status(system_id, component_id, _status, msg,  rs485_read_request->motor_id, rs485_read_request->address, rs485_read_request->length);
}

/**
 * @brief Send a rs485_read_request message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address to read
 * @param length  Number of bytes to read (1-64)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rs485_read_request_send(mavlink_channel_t chan, uint8_t motor_id, uint16_t address, uint8_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_READ_REQUEST, buf, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
#else
    mavlink_rs485_read_request_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_READ_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
#endif
}

/**
 * @brief Send a rs485_read_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rs485_read_request_send_struct(mavlink_channel_t chan, const mavlink_rs485_read_request_t* rs485_read_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rs485_read_request_send(chan, rs485_read_request->motor_id, rs485_read_request->address, rs485_read_request->length);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_READ_REQUEST, (const char *)rs485_read_request, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rs485_read_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint16_t address, uint8_t length)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_READ_REQUEST, buf, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
#else
    mavlink_rs485_read_request_t *packet = (mavlink_rs485_read_request_t *)msgbuf;
    packet->address = address;
    packet->motor_id = motor_id;
    packet->length = length;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_READ_REQUEST, (const char *)packet, MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN, MAVLINK_MSG_ID_RS485_READ_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE RS485_READ_REQUEST UNPACKING


/**
 * @brief Get field motor_id from rs485_read_request message
 *
 * @return  Motor ID (30-49 for RS485 motors)
 */
static inline uint8_t mavlink_msg_rs485_read_request_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field address from rs485_read_request message
 *
 * @return  Control table address to read
 */
static inline uint16_t mavlink_msg_rs485_read_request_get_address(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field length from rs485_read_request message
 *
 * @return  Number of bytes to read (1-64)
 */
static inline uint8_t mavlink_msg_rs485_read_request_get_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Decode a rs485_read_request message into a struct
 *
 * @param msg The message to decode
 * @param rs485_read_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_rs485_read_request_decode(const mavlink_message_t* msg, mavlink_rs485_read_request_t* rs485_read_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rs485_read_request->address = mavlink_msg_rs485_read_request_get_address(msg);
    rs485_read_request->motor_id = mavlink_msg_rs485_read_request_get_motor_id(msg);
    rs485_read_request->length = mavlink_msg_rs485_read_request_get_length(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN;
        memset(rs485_read_request, 0, MAVLINK_MSG_ID_RS485_READ_REQUEST_LEN);
    memcpy(rs485_read_request, _MAV_PAYLOAD(msg), len);
#endif
}
