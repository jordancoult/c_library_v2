#pragma once
// MESSAGE MORPH_STATUS PACKING

#define MAVLINK_MSG_ID_MORPH_STATUS 229


typedef struct __mavlink_morph_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.*/
 float angles[4]; /*< [deg] Angles of arms with the drone body. 45 degrees is neutral quad position, and counterclockwise is positive.*/
 float raw_cg[2]; /*< [m] Realtime calculated center of gravity, before any filtering/processing*/
 float filt_cg[2]; /*< [m] Post-processed center of gravity signal*/
 float ct[2]; /*< [m] CT (center of thrust) is the unweighted 2-D average of the 4 motor positions*/
 uint8_t mode; /*<  Mode indicating the current morphing functionality*/
 uint8_t shutter_open; /*<  Boolean indicating open/closed shutter. The shutter allows/denies updating of filt_cg depending upon the stability/acceleration of the vehicle*/
} mavlink_morph_status_t;

#define MAVLINK_MSG_ID_MORPH_STATUS_LEN 50
#define MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN 50
#define MAVLINK_MSG_ID_229_LEN 50
#define MAVLINK_MSG_ID_229_MIN_LEN 50

#define MAVLINK_MSG_ID_MORPH_STATUS_CRC 206
#define MAVLINK_MSG_ID_229_CRC 206

#define MAVLINK_MSG_MORPH_STATUS_FIELD_ANGLES_LEN 4
#define MAVLINK_MSG_MORPH_STATUS_FIELD_RAW_CG_LEN 2
#define MAVLINK_MSG_MORPH_STATUS_FIELD_FILT_CG_LEN 2
#define MAVLINK_MSG_MORPH_STATUS_FIELD_CT_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MORPH_STATUS { \
    229, \
    "MORPH_STATUS", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_morph_status_t, time_usec) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_morph_status_t, mode) }, \
         { "angles", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_morph_status_t, angles) }, \
         { "raw_cg", NULL, MAVLINK_TYPE_FLOAT, 2, 24, offsetof(mavlink_morph_status_t, raw_cg) }, \
         { "filt_cg", NULL, MAVLINK_TYPE_FLOAT, 2, 32, offsetof(mavlink_morph_status_t, filt_cg) }, \
         { "ct", NULL, MAVLINK_TYPE_FLOAT, 2, 40, offsetof(mavlink_morph_status_t, ct) }, \
         { "shutter_open", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_morph_status_t, shutter_open) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MORPH_STATUS { \
    "MORPH_STATUS", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_morph_status_t, time_usec) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_morph_status_t, mode) }, \
         { "angles", NULL, MAVLINK_TYPE_FLOAT, 4, 8, offsetof(mavlink_morph_status_t, angles) }, \
         { "raw_cg", NULL, MAVLINK_TYPE_FLOAT, 2, 24, offsetof(mavlink_morph_status_t, raw_cg) }, \
         { "filt_cg", NULL, MAVLINK_TYPE_FLOAT, 2, 32, offsetof(mavlink_morph_status_t, filt_cg) }, \
         { "ct", NULL, MAVLINK_TYPE_FLOAT, 2, 40, offsetof(mavlink_morph_status_t, ct) }, \
         { "shutter_open", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_morph_status_t, shutter_open) }, \
         } \
}
#endif

/**
 * @brief Pack a morph_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param mode  Mode indicating the current morphing functionality
 * @param angles [deg] Angles of arms with the drone body. 45 degrees is neutral quad position, and counterclockwise is positive.
 * @param raw_cg [m] Realtime calculated center of gravity, before any filtering/processing
 * @param filt_cg [m] Post-processed center of gravity signal
 * @param ct [m] CT (center of thrust) is the unweighted 2-D average of the 4 motor positions
 * @param shutter_open  Boolean indicating open/closed shutter. The shutter allows/denies updating of filt_cg depending upon the stability/acceleration of the vehicle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_morph_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, uint8_t mode, const float *angles, const float *raw_cg, const float *filt_cg, const float *ct, uint8_t shutter_open)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MORPH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 48, mode);
    _mav_put_uint8_t(buf, 49, shutter_open);
    _mav_put_float_array(buf, 8, angles, 4);
    _mav_put_float_array(buf, 24, raw_cg, 2);
    _mav_put_float_array(buf, 32, filt_cg, 2);
    _mav_put_float_array(buf, 40, ct, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
#else
    mavlink_morph_status_t packet;
    packet.time_usec = time_usec;
    packet.mode = mode;
    packet.shutter_open = shutter_open;
    mav_array_memcpy(packet.angles, angles, sizeof(float)*4);
    mav_array_memcpy(packet.raw_cg, raw_cg, sizeof(float)*2);
    mav_array_memcpy(packet.filt_cg, filt_cg, sizeof(float)*2);
    mav_array_memcpy(packet.ct, ct, sizeof(float)*2);
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
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param mode  Mode indicating the current morphing functionality
 * @param angles [deg] Angles of arms with the drone body. 45 degrees is neutral quad position, and counterclockwise is positive.
 * @param raw_cg [m] Realtime calculated center of gravity, before any filtering/processing
 * @param filt_cg [m] Post-processed center of gravity signal
 * @param ct [m] CT (center of thrust) is the unweighted 2-D average of the 4 motor positions
 * @param shutter_open  Boolean indicating open/closed shutter. The shutter allows/denies updating of filt_cg depending upon the stability/acceleration of the vehicle
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_morph_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,uint8_t mode,const float *angles,const float *raw_cg,const float *filt_cg,const float *ct,uint8_t shutter_open)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MORPH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 48, mode);
    _mav_put_uint8_t(buf, 49, shutter_open);
    _mav_put_float_array(buf, 8, angles, 4);
    _mav_put_float_array(buf, 24, raw_cg, 2);
    _mav_put_float_array(buf, 32, filt_cg, 2);
    _mav_put_float_array(buf, 40, ct, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
#else
    mavlink_morph_status_t packet;
    packet.time_usec = time_usec;
    packet.mode = mode;
    packet.shutter_open = shutter_open;
    mav_array_memcpy(packet.angles, angles, sizeof(float)*4);
    mav_array_memcpy(packet.raw_cg, raw_cg, sizeof(float)*2);
    mav_array_memcpy(packet.filt_cg, filt_cg, sizeof(float)*2);
    mav_array_memcpy(packet.ct, ct, sizeof(float)*2);
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
    return mavlink_msg_morph_status_pack(system_id, component_id, msg, morph_status->time_usec, morph_status->mode, morph_status->angles, morph_status->raw_cg, morph_status->filt_cg, morph_status->ct, morph_status->shutter_open);
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
    return mavlink_msg_morph_status_pack_chan(system_id, component_id, chan, msg, morph_status->time_usec, morph_status->mode, morph_status->angles, morph_status->raw_cg, morph_status->filt_cg, morph_status->ct, morph_status->shutter_open);
}

/**
 * @brief Send a morph_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 * @param mode  Mode indicating the current morphing functionality
 * @param angles [deg] Angles of arms with the drone body. 45 degrees is neutral quad position, and counterclockwise is positive.
 * @param raw_cg [m] Realtime calculated center of gravity, before any filtering/processing
 * @param filt_cg [m] Post-processed center of gravity signal
 * @param ct [m] CT (center of thrust) is the unweighted 2-D average of the 4 motor positions
 * @param shutter_open  Boolean indicating open/closed shutter. The shutter allows/denies updating of filt_cg depending upon the stability/acceleration of the vehicle
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_morph_status_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t mode, const float *angles, const float *raw_cg, const float *filt_cg, const float *ct, uint8_t shutter_open)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MORPH_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 48, mode);
    _mav_put_uint8_t(buf, 49, shutter_open);
    _mav_put_float_array(buf, 8, angles, 4);
    _mav_put_float_array(buf, 24, raw_cg, 2);
    _mav_put_float_array(buf, 32, filt_cg, 2);
    _mav_put_float_array(buf, 40, ct, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, buf, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#else
    mavlink_morph_status_t packet;
    packet.time_usec = time_usec;
    packet.mode = mode;
    packet.shutter_open = shutter_open;
    mav_array_memcpy(packet.angles, angles, sizeof(float)*4);
    mav_array_memcpy(packet.raw_cg, raw_cg, sizeof(float)*2);
    mav_array_memcpy(packet.filt_cg, filt_cg, sizeof(float)*2);
    mav_array_memcpy(packet.ct, ct, sizeof(float)*2);
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
    mavlink_msg_morph_status_send(chan, morph_status->time_usec, morph_status->mode, morph_status->angles, morph_status->raw_cg, morph_status->filt_cg, morph_status->ct, morph_status->shutter_open);
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
static inline void mavlink_msg_morph_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, uint8_t mode, const float *angles, const float *raw_cg, const float *filt_cg, const float *ct, uint8_t shutter_open)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint8_t(buf, 48, mode);
    _mav_put_uint8_t(buf, 49, shutter_open);
    _mav_put_float_array(buf, 8, angles, 4);
    _mav_put_float_array(buf, 24, raw_cg, 2);
    _mav_put_float_array(buf, 32, filt_cg, 2);
    _mav_put_float_array(buf, 40, ct, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, buf, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#else
    mavlink_morph_status_t *packet = (mavlink_morph_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->mode = mode;
    packet->shutter_open = shutter_open;
    mav_array_memcpy(packet->angles, angles, sizeof(float)*4);
    mav_array_memcpy(packet->raw_cg, raw_cg, sizeof(float)*2);
    mav_array_memcpy(packet->filt_cg, filt_cg, sizeof(float)*2);
    mav_array_memcpy(packet->ct, ct, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MORPH_STATUS, (const char *)packet, MAVLINK_MSG_ID_MORPH_STATUS_MIN_LEN, MAVLINK_MSG_ID_MORPH_STATUS_LEN, MAVLINK_MSG_ID_MORPH_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MORPH_STATUS UNPACKING


/**
 * @brief Get field time_usec from morph_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
 */
static inline uint64_t mavlink_msg_morph_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field mode from morph_status message
 *
 * @return  Mode indicating the current morphing functionality
 */
static inline uint8_t mavlink_msg_morph_status_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field angles from morph_status message
 *
 * @return [deg] Angles of arms with the drone body. 45 degrees is neutral quad position, and counterclockwise is positive.
 */
static inline uint16_t mavlink_msg_morph_status_get_angles(const mavlink_message_t* msg, float *angles)
{
    return _MAV_RETURN_float_array(msg, angles, 4,  8);
}

/**
 * @brief Get field raw_cg from morph_status message
 *
 * @return [m] Realtime calculated center of gravity, before any filtering/processing
 */
static inline uint16_t mavlink_msg_morph_status_get_raw_cg(const mavlink_message_t* msg, float *raw_cg)
{
    return _MAV_RETURN_float_array(msg, raw_cg, 2,  24);
}

/**
 * @brief Get field filt_cg from morph_status message
 *
 * @return [m] Post-processed center of gravity signal
 */
static inline uint16_t mavlink_msg_morph_status_get_filt_cg(const mavlink_message_t* msg, float *filt_cg)
{
    return _MAV_RETURN_float_array(msg, filt_cg, 2,  32);
}

/**
 * @brief Get field ct from morph_status message
 *
 * @return [m] CT (center of thrust) is the unweighted 2-D average of the 4 motor positions
 */
static inline uint16_t mavlink_msg_morph_status_get_ct(const mavlink_message_t* msg, float *ct)
{
    return _MAV_RETURN_float_array(msg, ct, 2,  40);
}

/**
 * @brief Get field shutter_open from morph_status message
 *
 * @return  Boolean indicating open/closed shutter. The shutter allows/denies updating of filt_cg depending upon the stability/acceleration of the vehicle
 */
static inline uint8_t mavlink_msg_morph_status_get_shutter_open(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
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
    morph_status->time_usec = mavlink_msg_morph_status_get_time_usec(msg);
    mavlink_msg_morph_status_get_angles(msg, morph_status->angles);
    mavlink_msg_morph_status_get_raw_cg(msg, morph_status->raw_cg);
    mavlink_msg_morph_status_get_filt_cg(msg, morph_status->filt_cg);
    mavlink_msg_morph_status_get_ct(msg, morph_status->ct);
    morph_status->mode = mavlink_msg_morph_status_get_mode(msg);
    morph_status->shutter_open = mavlink_msg_morph_status_get_shutter_open(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MORPH_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MORPH_STATUS_LEN;
        memset(morph_status, 0, MAVLINK_MSG_ID_MORPH_STATUS_LEN);
    memcpy(morph_status, _MAV_PAYLOAD(msg), len);
#endif
}
