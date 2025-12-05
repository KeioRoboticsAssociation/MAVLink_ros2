#pragma once
// MESSAGE RS485_WRITE_RESPONSE PACKING

#define MAVLINK_MSG_ID_RS485_WRITE_RESPONSE 12010


typedef struct __mavlink_rs485_write_response_t {
 uint16_t address; /*<  Control table address that was written*/
 uint8_t motor_id; /*<  Motor ID (30-49 for RS485 motors)*/
 uint8_t length; /*<  Number of bytes written*/
 uint8_t status; /*<  Status code (0=success, see error_code_t for errors)*/
 uint8_t rs485_error; /*<  RS485 protocol error code (0=no error)*/
} mavlink_rs485_write_response_t;

#define MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN 6
#define MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN 6
#define MAVLINK_MSG_ID_12010_LEN 6
#define MAVLINK_MSG_ID_12010_MIN_LEN 6

#define MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC 194
#define MAVLINK_MSG_ID_12010_CRC 194



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RS485_WRITE_RESPONSE { \
    12010, \
    "RS485_WRITE_RESPONSE", \
    5, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rs485_write_response_t, motor_id) }, \
         { "address", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rs485_write_response_t, address) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_rs485_write_response_t, length) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_rs485_write_response_t, status) }, \
         { "rs485_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_rs485_write_response_t, rs485_error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RS485_WRITE_RESPONSE { \
    "RS485_WRITE_RESPONSE", \
    5, \
    {  { "motor_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rs485_write_response_t, motor_id) }, \
         { "address", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rs485_write_response_t, address) }, \
         { "length", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_rs485_write_response_t, length) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_rs485_write_response_t, status) }, \
         { "rs485_error", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_rs485_write_response_t, rs485_error) }, \
         } \
}
#endif

/**
 * @brief Pack a rs485_write_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address that was written
 * @param length  Number of bytes written
 * @param status  Status code (0=success, see error_code_t for errors)
 * @param rs485_error  RS485 protocol error code (0=no error)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_write_response_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t motor_id, uint16_t address, uint8_t length, uint8_t status, uint8_t rs485_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);
    _mav_put_uint8_t(buf, 4, status);
    _mav_put_uint8_t(buf, 5, rs485_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#else
    mavlink_rs485_write_response_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;
    packet.status = status;
    packet.rs485_error = rs485_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_WRITE_RESPONSE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
}

/**
 * @brief Pack a rs485_write_response message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address that was written
 * @param length  Number of bytes written
 * @param status  Status code (0=success, see error_code_t for errors)
 * @param rs485_error  RS485 protocol error code (0=no error)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_write_response_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t motor_id, uint16_t address, uint8_t length, uint8_t status, uint8_t rs485_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);
    _mav_put_uint8_t(buf, 4, status);
    _mav_put_uint8_t(buf, 5, rs485_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#else
    mavlink_rs485_write_response_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;
    packet.status = status;
    packet.rs485_error = rs485_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_WRITE_RESPONSE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#endif
}

/**
 * @brief Pack a rs485_write_response message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address that was written
 * @param length  Number of bytes written
 * @param status  Status code (0=success, see error_code_t for errors)
 * @param rs485_error  RS485 protocol error code (0=no error)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rs485_write_response_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t motor_id,uint16_t address,uint8_t length,uint8_t status,uint8_t rs485_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);
    _mav_put_uint8_t(buf, 4, status);
    _mav_put_uint8_t(buf, 5, rs485_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#else
    mavlink_rs485_write_response_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;
    packet.status = status;
    packet.rs485_error = rs485_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RS485_WRITE_RESPONSE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
}

/**
 * @brief Encode a rs485_write_response struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rs485_write_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_write_response_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rs485_write_response_t* rs485_write_response)
{
    return mavlink_msg_rs485_write_response_pack(system_id, component_id, msg, rs485_write_response->motor_id, rs485_write_response->address, rs485_write_response->length, rs485_write_response->status, rs485_write_response->rs485_error);
}

/**
 * @brief Encode a rs485_write_response struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rs485_write_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_write_response_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rs485_write_response_t* rs485_write_response)
{
    return mavlink_msg_rs485_write_response_pack_chan(system_id, component_id, chan, msg, rs485_write_response->motor_id, rs485_write_response->address, rs485_write_response->length, rs485_write_response->status, rs485_write_response->rs485_error);
}

/**
 * @brief Encode a rs485_write_response struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param rs485_write_response C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rs485_write_response_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_rs485_write_response_t* rs485_write_response)
{
    return mavlink_msg_rs485_write_response_pack_status(system_id, component_id, _status, msg,  rs485_write_response->motor_id, rs485_write_response->address, rs485_write_response->length, rs485_write_response->status, rs485_write_response->rs485_error);
}

/**
 * @brief Send a rs485_write_response message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_id  Motor ID (30-49 for RS485 motors)
 * @param address  Control table address that was written
 * @param length  Number of bytes written
 * @param status  Status code (0=success, see error_code_t for errors)
 * @param rs485_error  RS485 protocol error code (0=no error)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rs485_write_response_send(mavlink_channel_t chan, uint8_t motor_id, uint16_t address, uint8_t length, uint8_t status, uint8_t rs485_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN];
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);
    _mav_put_uint8_t(buf, 4, status);
    _mav_put_uint8_t(buf, 5, rs485_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE, buf, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
#else
    mavlink_rs485_write_response_t packet;
    packet.address = address;
    packet.motor_id = motor_id;
    packet.length = length;
    packet.status = status;
    packet.rs485_error = rs485_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE, (const char *)&packet, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
#endif
}

/**
 * @brief Send a rs485_write_response message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rs485_write_response_send_struct(mavlink_channel_t chan, const mavlink_rs485_write_response_t* rs485_write_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rs485_write_response_send(chan, rs485_write_response->motor_id, rs485_write_response->address, rs485_write_response->length, rs485_write_response->status, rs485_write_response->rs485_error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE, (const char *)rs485_write_response, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
#endif
}

#if MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rs485_write_response_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t motor_id, uint16_t address, uint8_t length, uint8_t status, uint8_t rs485_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, address);
    _mav_put_uint8_t(buf, 2, motor_id);
    _mav_put_uint8_t(buf, 3, length);
    _mav_put_uint8_t(buf, 4, status);
    _mav_put_uint8_t(buf, 5, rs485_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE, buf, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
#else
    mavlink_rs485_write_response_t *packet = (mavlink_rs485_write_response_t *)msgbuf;
    packet->address = address;
    packet->motor_id = motor_id;
    packet->length = length;
    packet->status = status;
    packet->rs485_error = rs485_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE, (const char *)packet, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_CRC);
#endif
}
#endif

#endif

// MESSAGE RS485_WRITE_RESPONSE UNPACKING


/**
 * @brief Get field motor_id from rs485_write_response message
 *
 * @return  Motor ID (30-49 for RS485 motors)
 */
static inline uint8_t mavlink_msg_rs485_write_response_get_motor_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field address from rs485_write_response message
 *
 * @return  Control table address that was written
 */
static inline uint16_t mavlink_msg_rs485_write_response_get_address(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field length from rs485_write_response message
 *
 * @return  Number of bytes written
 */
static inline uint8_t mavlink_msg_rs485_write_response_get_length(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field status from rs485_write_response message
 *
 * @return  Status code (0=success, see error_code_t for errors)
 */
static inline uint8_t mavlink_msg_rs485_write_response_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field rs485_error from rs485_write_response message
 *
 * @return  RS485 protocol error code (0=no error)
 */
static inline uint8_t mavlink_msg_rs485_write_response_get_rs485_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Decode a rs485_write_response message into a struct
 *
 * @param msg The message to decode
 * @param rs485_write_response C-struct to decode the message contents into
 */
static inline void mavlink_msg_rs485_write_response_decode(const mavlink_message_t* msg, mavlink_rs485_write_response_t* rs485_write_response)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rs485_write_response->address = mavlink_msg_rs485_write_response_get_address(msg);
    rs485_write_response->motor_id = mavlink_msg_rs485_write_response_get_motor_id(msg);
    rs485_write_response->length = mavlink_msg_rs485_write_response_get_length(msg);
    rs485_write_response->status = mavlink_msg_rs485_write_response_get_status(msg);
    rs485_write_response->rs485_error = mavlink_msg_rs485_write_response_get_rs485_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN? msg->len : MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN;
        memset(rs485_write_response, 0, MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_LEN);
    memcpy(rs485_write_response, _MAV_PAYLOAD(msg), len);
#endif
}
