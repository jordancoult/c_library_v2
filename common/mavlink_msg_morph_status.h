#pragma once
// MESSAGE MORPH_STATUS PACKING

#define MAVLINK_MSG_ID_MORPH_STATUS 229

MAVPACKED(
typedef struct __mavlink_morph_status_t {
 uint8_t mode; /*<   */
}) mavlink_morph_status_t;

#define MAVLINK_MSG_ID_MORPH_STATUS_LEN 1
#define MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN 1
#define MAVLINK_MSG_ID_229_LEN 1
#define MAVLINK_MSG_ID_229_MIN_LEN 1

#define MAVLINK_MSG_ID_MORPH_STATUS_CRC 249
#define MAVLINK_MSG_ID_229_CRC 249



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MORPH_STATUS { \
    229, \
    "MORPH_STATUS", \
    1, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_morph_status_t, mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MORPH_STATUS { \
    "MORPH_STATUS", \
    1, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_morph_status_t, mode) }, \
         } \
}
#endif

/**
 * @brief Pack a morph_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_morph_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MORPH_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
#else
    mavlink_morph_status_t packet;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MORPH_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
}

/**
 * @brief Pack a morph_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_morph_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MORPH_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
#else
    mavlink_morph_status_t packet;
    packet.mode = mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MORPH_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
}

/**
 * @brief Encode a morph_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param morph_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_morph_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_morph_status_t* morph_status)
{
    return mavlink_msg_morph_status_pack(system_id, component_id, msg, morph_status->mode);
}

/**
 * @brief Encode a morph_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param morph_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_morph_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_morph_status_t* morph_status)
{
    return mavlink_msg_morph_status_pack_chan(system_id, component_id, chan, msg, morph_status->mode);
}

/**
 * @brief Send a morph_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode   
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_morph_status_send(mavlink_channel_t chan, uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MORPH_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, buf, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#else
    mavlink_morph_status_t packet;
    packet.mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#endif
}

/**
 * @brief Send a morph_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_morph_status_send_struct(mavlink_channel_t chan, const mavlink_morph_status_t* morph_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_morph_status_send(chan, morph_status->mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, (const char *)morph_status, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MORPH_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_morph_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, buf, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#else
    mavlink_morph_status_t *packet = (mavlink_morph_status_t *)msgbuf;
    packet->mode = mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, (const char *)packet, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MORPH_STATUS UNPACKING


/**
 * @brief Get field mode from morph_status message
 *
 * @return   
 */
static inline uint8_t mavlink_msg_morph_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a morph_status message into a struct
 *
 * @param msg The message to decode
 * @param morph_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_morph_status_decode(const mavlink_message_t* msg, mavlink_morph_status_t* morph_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    morph_status->mode = mavlink_msg_morph_status_get_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MORPH_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MORPH_STATUS_LEN;
        memset(morph_status, 0, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
    memcpy(morph_status, _MAV_PAYLOAD(msg), len);
#endif
}
