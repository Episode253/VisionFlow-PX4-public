#pragma once
// MESSAGE MAV_GS PACKING

#define MAVLINK_MSG_ID_MAV_GS 418


typedef struct __mavlink_mav_gs_t {
 float gs[3]; /*<  gs-vector.*/
} mavlink_mav_gs_t;

#define MAVLINK_MSG_ID_MAV_GS_LEN 12
#define MAVLINK_MSG_ID_MAV_GS_MIN_LEN 12
#define MAVLINK_MSG_ID_418_LEN 12
#define MAVLINK_MSG_ID_418_MIN_LEN 12

#define MAVLINK_MSG_ID_MAV_GS_CRC 99
#define MAVLINK_MSG_ID_418_CRC 99

#define MAVLINK_MSG_MAV_GS_FIELD_GS_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAV_GS { \
    418, \
    "MAV_GS", \
    1, \
    {  { "gs", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_mav_gs_t, gs) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAV_GS { \
    "MAV_GS", \
    1, \
    {  { "gs", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_mav_gs_t, gs) }, \
         } \
}
#endif

/**
 * @brief Pack a mav_gs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gs  gs-vector.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_gs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_GS_LEN];

    _mav_put_float_array(buf, 0, gs, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_GS_LEN);
#else
    mavlink_mav_gs_t packet;

    mav_array_memcpy(packet.gs, gs, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_GS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_GS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
}

/**
 * @brief Pack a mav_gs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param gs  gs-vector.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_gs_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const float *gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_GS_LEN];

    _mav_put_float_array(buf, 0, gs, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_GS_LEN);
#else
    mavlink_mav_gs_t packet;

    mav_array_memcpy(packet.gs, gs, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_GS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_GS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN);
#endif
}

/**
 * @brief Pack a mav_gs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gs  gs-vector.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mav_gs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_GS_LEN];

    _mav_put_float_array(buf, 0, gs, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAV_GS_LEN);
#else
    mavlink_mav_gs_t packet;

    mav_array_memcpy(packet.gs, gs, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAV_GS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAV_GS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
}

/**
 * @brief Encode a mav_gs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mav_gs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_gs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mav_gs_t* mav_gs)
{
    return mavlink_msg_mav_gs_pack(system_id, component_id, msg, mav_gs->gs);
}

/**
 * @brief Encode a mav_gs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mav_gs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_gs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mav_gs_t* mav_gs)
{
    return mavlink_msg_mav_gs_pack_chan(system_id, component_id, chan, msg, mav_gs->gs);
}

/**
 * @brief Encode a mav_gs struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mav_gs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mav_gs_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mav_gs_t* mav_gs)
{
    return mavlink_msg_mav_gs_pack_status(system_id, component_id, _status, msg,  mav_gs->gs);
}

/**
 * @brief Send a mav_gs message
 * @param chan MAVLink channel to send the message
 *
 * @param gs  gs-vector.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mav_gs_send(mavlink_channel_t chan, const float *gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAV_GS_LEN];

    _mav_put_float_array(buf, 0, gs, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GS, buf, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
#else
    mavlink_mav_gs_t packet;

    mav_array_memcpy(packet.gs, gs, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GS, (const char *)&packet, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
#endif
}

/**
 * @brief Send a mav_gs message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mav_gs_send_struct(mavlink_channel_t chan, const mavlink_mav_gs_t* mav_gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mav_gs_send(chan, mav_gs->gs);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GS, (const char *)mav_gs, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAV_GS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mav_gs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, gs, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GS, buf, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
#else
    mavlink_mav_gs_t *packet = (mavlink_mav_gs_t *)msgbuf;

    mav_array_memcpy(packet->gs, gs, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAV_GS, (const char *)packet, MAVLINK_MSG_ID_MAV_GS_MIN_LEN, MAVLINK_MSG_ID_MAV_GS_LEN, MAVLINK_MSG_ID_MAV_GS_CRC);
#endif
}
#endif

#endif

// MESSAGE MAV_GS UNPACKING


/**
 * @brief Get field gs from mav_gs message
 *
 * @return  gs-vector.
 */
static inline uint16_t mavlink_msg_mav_gs_get_gs(const mavlink_message_t* msg, float *gs)
{
    return _MAV_RETURN_float_array(msg, gs, 3,  0);
}

/**
 * @brief Decode a mav_gs message into a struct
 *
 * @param msg The message to decode
 * @param mav_gs C-struct to decode the message contents into
 */
static inline void mavlink_msg_mav_gs_decode(const mavlink_message_t* msg, mavlink_mav_gs_t* mav_gs)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mav_gs_get_gs(msg, mav_gs->gs);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAV_GS_LEN? msg->len : MAVLINK_MSG_ID_MAV_GS_LEN;
        memset(mav_gs, 0, MAVLINK_MSG_ID_MAV_GS_LEN);
    memcpy(mav_gs, _MAV_PAYLOAD(msg), len);
#endif
}
