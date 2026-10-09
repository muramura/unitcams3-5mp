#pragma once

#include <stdint.h>
#include <string.h>

#define MAVLINK_V2_MAGIC 0xFD

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ 20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN 20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_CRC_EXTRA 214

#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST 21
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN 2
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST_CRC_EXTRA 159

#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 25
#define MAVLINK_MSG_ID_PARAM_VALUE_CRC_EXTRA 220

#define MAVLINK_MSG_ID_PARAM_SET 23
#define MAVLINK_MSG_ID_PARAM_SET_LEN 23
#define MAVLINK_MSG_ID_PARAM_SET_CRC_EXTRA 168

#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33
#define MAVLINK_MSG_ID_COMMAND_LONG_CRC_EXTRA 152

#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_COMMAND_ACK_LEN 3
#define MAVLINK_MSG_ID_COMMAND_ACK_CRC_EXTRA 143

#define MAV_COMP_ID_OPTICAL_FLOW 197

// Standard MAV_PARAM_TYPE
#define MAV_PARAM_TYPE_UINT8   1
#define MAV_PARAM_TYPE_INT8    2
#define MAV_PARAM_TYPE_UINT16  3
#define MAV_PARAM_TYPE_INT16   4
#define MAV_PARAM_TYPE_UINT32  5
#define MAV_PARAM_TYPE_INT32   6
#define MAV_PARAM_TYPE_REAL32  9

// Standard MAV_CMD
#define MAV_CMD_PREFLIGHT_STORAGE 245
#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_DENIED   2

// CRC accumulator for MAVLink X.25 / MCRF4XX
static inline void mavlink_crc_accumulate_param(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

/**
 * @brief Pack a PARAM_VALUE message (MAVLink v2)
 *
 * Wire format (payload 25 bytes):
 *  0..3:  float param_value (4 bytes)
 *  4..5:  uint16_t param_count (2 bytes)
 *  6..7:  uint16_t param_index (2 bytes)
 *  8..23: char param_id[16] (16 bytes, null terminated or padded)
 *  24:    uint8_t param_type (1 byte)
 *
 * @param buf Output buffer (at least 37 bytes)
 * @param seq Pointer to running sequence number
 * @param param_id 16-character parameter identifier
 * @param param_value Float value
 * @param param_type MAV_PARAM_TYPE
 * @param param_count Total number of parameters
 * @param param_index Zero-based index of this parameter
 * @return Total packet length (37 bytes)
 */
static inline uint16_t mavlink_pack_param_value_v2(
    uint8_t *buf,
    uint8_t *seq,
    const char *param_id,
    float param_value,
    uint8_t param_type,
    uint16_t param_count,
    uint16_t param_index)
{
    // MAVLink v2 Header (10 bytes)
    buf[0] = MAVLINK_V2_MAGIC;                  // 0xFD
    buf[1] = MAVLINK_MSG_ID_PARAM_VALUE_LEN;    // 25 bytes payload
    buf[2] = 0x00;                              // Incompat flags
    buf[3] = 0x00;                              // Compat flags
    buf[4] = (*seq)++;                          // Sequence number
    buf[5] = 1;                                 // System ID (matches vehicle sysid 1)
    buf[6] = MAV_COMP_ID_OPTICAL_FLOW;          // Component ID: 197
    buf[7] = (uint8_t)(MAVLINK_MSG_ID_PARAM_VALUE & 0xFF);         // 22
    buf[8] = (uint8_t)((MAVLINK_MSG_ID_PARAM_VALUE >> 8) & 0xFF);  // 0
    buf[9] = (uint8_t)((MAVLINK_MSG_ID_PARAM_VALUE >> 16) & 0xFF); // 0

    // Payload (25 bytes)
    // 0..3: param_value (float, 4)
    memcpy(&buf[10], &param_value, 4);

    // 4..5: param_count (uint16_t, 2)
    buf[14] = (uint8_t)(param_count & 0xFF);
    buf[15] = (uint8_t)((param_count >> 8) & 0xFF);

    // 6..7: param_index (uint16_t, 2)
    buf[16] = (uint8_t)(param_index & 0xFF);
    buf[17] = (uint8_t)((param_index >> 8) & 0xFF);

    // 8..23: param_id (char[16])
    memset(&buf[18], 0, 16);
    if (param_id) {
        strncpy((char *)&buf[18], param_id, 16);
    }

    // 24: param_type (uint8_t, 1)
    buf[34] = param_type;

    // CRC Calculation
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 35; i++) {
        mavlink_crc_accumulate_param(buf[i], &crc);
    }
    mavlink_crc_accumulate_param(MAVLINK_MSG_ID_PARAM_VALUE_CRC_EXTRA, &crc);

    buf[35] = (uint8_t)(crc & 0xFF);
    buf[36] = (uint8_t)((crc >> 8) & 0xFF);

    return 37;
}

/**
 * @brief Pack a COMMAND_ACK message (MAVLink v2)
 *
 * Wire format (payload 3 bytes for standard v1/v2 ACK):
 *  0..1: uint16_t command
 *  2:    uint8_t result
 *
 * @param buf Output buffer (at least 15 bytes)
 * @param seq Pointer to running sequence number
 * @param command MAV_CMD being acknowledged
 * @param result MAV_RESULT
 * @return Total packet length (15 bytes)
 */
static inline uint16_t mavlink_pack_command_ack_v2(
    uint8_t *buf,
    uint8_t *seq,
    uint16_t command,
    uint8_t result)
{
    // MAVLink v2 Header (10 bytes)
    buf[0] = MAVLINK_V2_MAGIC;                  // 0xFD
    buf[1] = MAVLINK_MSG_ID_COMMAND_ACK_LEN;    // 3 bytes payload
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = (*seq)++;
    buf[5] = 1;                                 // System ID: 1
    buf[6] = MAV_COMP_ID_OPTICAL_FLOW;          // Component ID: 197
    buf[7] = (uint8_t)(MAVLINK_MSG_ID_COMMAND_ACK & 0xFF);         // 77
    buf[8] = (uint8_t)((MAVLINK_MSG_ID_COMMAND_ACK >> 8) & 0xFF);  // 0
    buf[9] = (uint8_t)((MAVLINK_MSG_ID_COMMAND_ACK >> 16) & 0xFF); // 0

    // Payload (3 bytes)
    buf[10] = (uint8_t)(command & 0xFF);
    buf[11] = (uint8_t)((command >> 8) & 0xFF);
    buf[12] = result;

    // CRC Calculation
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 13; i++) {
        mavlink_crc_accumulate_param(buf[i], &crc);
    }
    mavlink_crc_accumulate_param(MAVLINK_MSG_ID_COMMAND_ACK_CRC_EXTRA, &crc);

    buf[13] = (uint8_t)(crc & 0xFF);
    buf[14] = (uint8_t)((crc >> 8) & 0xFF);

    return 15;
}
