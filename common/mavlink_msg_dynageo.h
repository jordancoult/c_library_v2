#pragma once
// MESSAGE DYNAGEO PACKING

#define MAVLINK_MSG_ID_DYNAGEO 228


typedef struct __mavlink_dynageo_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 float roll[4]; /*<  Roll scaling for top motors of mixer*/
 float pitch[4]; /*<  Pitch scaling for top motors of mixer*/
 uint8_t failed_arm; /*<  int, 0-4, indicating which arm failed (front_R, rear_L, front_L, rear_R, none)*/
} mavlink_dynageo_t;

#define MAVLINK_MSG_ID_DYNAGEO_LEN 41
#define MAVLINK_MSG_ID_DYNAGEO_MIN_LEN 41
#define MAVLINK_MSG_ID_228_LEN 41
#define MAVLINK_MSG_ID_228_MIN_LEN 41

#define MAVLINK_MSG_ID_DYNAGEO_CRC 228
#define MAVLINK_MSG_ID_228_CRC 228

#define MAVLINK_MSG_DYNAGEO_FIELD_ROLL_LEN 4
#define MAVLINK_MSG_DYNAGEO_FIELD_PITCH_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DYNAGEO { \
    228, \
    "DYNAGEO", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_dynageo_t, time_usec) }, \
         { "failed_arm", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_dynageo_t, failed_arm) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_dynageo_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_dynageo_t, pitch) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DYNAGEO { \
    "DYNAGEO", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_dynageo_t, time_usec) }, \
         { "failed_arm", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_dynageo_t, failed_arm) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_dynageo_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 4, 24, offsetof(mavlink_dynageo_t, pitch) }, \
         } \
}
#endif

/**
 * @brief Pack a dynageo message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param failed_arm  int, 0-4, indicating which arm failed (front_R, rear_L, front_L, rear_R, none)
 * @param roll  Roll scaling for top motors of mixer
 * @param pitch  Pitch scaling for top motors of mixer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dynageo_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t failed_arm, const float *roll, const float *pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DYNAGEO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 40, failed_arm);
    _mav_put_float_array(buf, 8, roll, 4);
    _mav_put_float_array(buf, 24, pitch, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DYNAGEO_LEN);
#else
    mavlink_dynageo_t packet;
    packet.time_usec = time_usec;
    packet.failed_arm = failed_arm;
    mav_array_memcpy(packet.roll, roll, sizeof(float)*4);
    mav_array_memcpy(packet.pitch, pitch, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DYNAGEO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DYNAGEO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
}

/**
 * @brief Pack a dynageo message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param failed_arm  int, 0-4, indicating which arm failed (front_R, rear_L, front_L, rear_R, none)
 * @param roll  Roll scaling for top motors of mixer
 * @param pitch  Pitch scaling for top motors of mixer
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dynageo_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t failed_arm,const float *roll,const float *pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DYNAGEO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 40, failed_arm);
    _mav_put_float_array(buf, 8, roll, 4);
    _mav_put_float_array(buf, 24, pitch, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DYNAGEO_LEN);
#else
    mavlink_dynageo_t packet;
    packet.time_usec = time_usec;
    packet.failed_arm = failed_arm;
    mav_array_memcpy(packet.roll, roll, sizeof(float)*4);
    mav_array_memcpy(packet.pitch, pitch, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DYNAGEO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DYNAGEO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
}

/**
 * @brief Encode a dynageo struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dynageo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dynageo_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dynageo_t* dynageo)
{
    return mavlink_msg_dynageo_pack(system_id, component_id, msg, dynageo->time_usec, dynageo->failed_arm, dynageo->roll, dynageo->pitch);
}

/**
 * @brief Encode a dynageo struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dynageo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dynageo_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dynageo_t* dynageo)
{
    return mavlink_msg_dynageo_pack_chan(system_id, component_id, chan, msg, dynageo->time_usec, dynageo->failed_arm, dynageo->roll, dynageo->pitch);
}

/**
 * @brief Send a dynageo message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param failed_arm  int, 0-4, indicating which arm failed (front_R, rear_L, front_L, rear_R, none)
 * @param roll  Roll scaling for top motors of mixer
 * @param pitch  Pitch scaling for top motors of mixer
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dynageo_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t failed_arm, const float *roll, const float *pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DYNAGEO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 40, failed_arm);
    _mav_put_float_array(buf, 8, roll, 4);
    _mav_put_float_array(buf, 24, pitch, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DYNAGEO, buf, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
#else
    mavlink_dynageo_t packet;
    packet.time_usec = time_usec;
    packet.failed_arm = failed_arm;
    mav_array_memcpy(packet.roll, roll, sizeof(float)*4);
    mav_array_memcpy(packet.pitch, pitch, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DYNAGEO, (const char *)&packet, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
#endif
}

/**
 * @brief Send a dynageo message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_dynageo_send_struct(mavlink_channel_t chan, const mavlink_dynageo_t* dynageo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_dynageo_send(chan, dynageo->time_usec, dynageo->failed_arm, dynageo->roll, dynageo->pitch);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DYNAGEO, (const char *)dynageo, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
#endif
}

#if MAVLINK_MSG_ID_DYNAGEO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_dynageo_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t failed_arm, const float *roll, const float *pitch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 40, failed_arm);
    _mav_put_float_array(buf, 8, roll, 4);
    _mav_put_float_array(buf, 24, pitch, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DYNAGEO, buf, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
#else
    mavlink_dynageo_t *packet = (mavlink_dynageo_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->failed_arm = failed_arm;
    mav_array_memcpy(packet->roll, roll, sizeof(float)*4);
    mav_array_memcpy(packet->pitch, pitch, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DYNAGEO, (const char *)packet, MAVLINK_MSG_ID_DYNAGEO_MIN_LEN, MAVLINK_MSG_ID_DYNAGEO_LEN, MAVLINK_MSG_ID_DYNAGEO_CRC);
#endif
}
#endif

#endif

// MESSAGE DYNAGEO UNPACKING


/**
 * @brief Get field time_usec from dynageo message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_dynageo_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field failed_arm from dynageo message
 *
 * @return  int, 0-4, indicating which arm failed (front_R, rear_L, front_L, rear_R, none)
 */
static inline uint8_t mavlink_msg_dynageo_get_failed_arm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field roll from dynageo message
 *
 * @return  Roll scaling for top motors of mixer
 */
static inline uint16_t mavlink_msg_dynageo_get_roll(const mavlink_message_t* msg, float *roll)
{
    return _MAV_RETURN_float_array(msg, roll, 4,  8);
}

/**
 * @brief Get field pitch from dynageo message
 *
 * @return  Pitch scaling for top motors of mixer
 */
static inline uint16_t mavlink_msg_dynageo_get_pitch(const mavlink_message_t* msg, float *pitch)
{
    return _MAV_RETURN_float_array(msg, pitch, 4,  24);
}

/**
 * @brief Decode a dynageo message into a struct
 *
 * @param msg The message to decode
 * @param dynageo C-struct to decode the message contents into
 */
static inline void mavlink_msg_dynageo_decode(const mavlink_message_t* msg, mavlink_dynageo_t* dynageo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    dynageo->time_usec = mavlink_msg_dynageo_get_time_usec(msg);
    mavlink_msg_dynageo_get_roll(msg, dynageo->roll);
    mavlink_msg_dynageo_get_pitch(msg, dynageo->pitch);
    dynageo->failed_arm = mavlink_msg_dynageo_get_failed_arm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DYNAGEO_LEN? msg->len : MAVLINK_MSG_ID_DYNAGEO_LEN;
        memset(dynageo, 0, MAVLINK_MSG_ID_DYNAGEO_LEN);
    memcpy(dynageo, _MAV_PAYLOAD(msg), len);
#endif
}
