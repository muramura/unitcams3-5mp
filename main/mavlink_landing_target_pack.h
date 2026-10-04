#pragma once

#include <stdint.h>
#include <string.h>
#include "mavlink_optical_flow_pack.h"

#define MAVLINK_MSG_ID_LANDING_TARGET 149
#define MAVLINK_MSG_ID_LANDING_TARGET_LEN 60
#define MAVLINK_MSG_ID_LANDING_TARGET_CRC_EXTRA 200

// MAV_FRAME enum
#define MAV_FRAME_BODY_NED 8

// LANDING_TARGET_TYPE enum
#define LANDING_TARGET_TYPE_VISION_OTHER 1

/**
 * @brief Pack a LANDING_TARGET message (MAVLink v2)
 *
 * @param buf Output buffer (must be at least 72 bytes)
 * @param time_usec Timestamp in microseconds
 * @param target_num ID of the target (e.g. ArUco ID, 0 for primary landing pad)
 * @param angle_x Horizontal angle to target in radians (positive = right)
 * @param angle_y Vertical angle to target in radians (positive = down)
 * @param distance Estimated distance to target in meters (-1 if unknown)
 * @param size_x Size of target in radians along X-axis
 * @param size_y Size of target in radians along Y-axis
 * @param seq Pointer to running sequence number (incremented automatically)
 * @return Total packet length in bytes (72 bytes)
 */
static inline uint16_t mavlink_pack_landing_target_v2(
    uint8_t *buf,
    uint64_t time_usec,
    uint8_t target_num,
    float angle_x,
    float angle_y,
    float distance,
    float size_x,
    float size_y,
    uint8_t *seq)
{
    // MAVLink v2 Header (10 bytes)
    buf[0] = MAVLINK_V2_MAGIC;                  // 0xFD
    buf[1] = MAVLINK_MSG_ID_LANDING_TARGET_LEN; // 60 bytes payload
    buf[2] = 0x00;                              // Incompat flags
    buf[3] = 0x00;                              // Compat flags
    buf[4] = (*seq)++;                          // Sequence number
    buf[5] = 1;                                 // System ID
    buf[6] = 197;                               // Component ID
    buf[7] = (uint8_t)(MAVLINK_MSG_ID_LANDING_TARGET & 0xFF);         // 149
    buf[8] = (uint8_t)((MAVLINK_MSG_ID_LANDING_TARGET >> 8) & 0xFF);  // 0
    buf[9] = (uint8_t)((MAVLINK_MSG_ID_LANDING_TARGET >> 16) & 0xFF); // 0

    // Payload (60 bytes, little-endian)
    // 0: time_usec (uint64_t, 8)
    memcpy(&buf[10], &time_usec, 8);
    // 8: angle_x (float, 4)
    memcpy(&buf[18], &angle_x, 4);
    // 12: angle_y (float, 4)
    memcpy(&buf[22], &angle_y, 4);
    // 16: distance (float, 4)
    memcpy(&buf[26], &distance, 4);
    // 20: size_x (float, 4)
    memcpy(&buf[30], &size_x, 4);
    // 24: size_y (float, 4)
    memcpy(&buf[34], &size_y, 4);
    // 28: target_num (uint8_t, 1)
    buf[38] = target_num;
    // 29: frame (uint8_t, 1): MAV_FRAME_BODY_NED (8)
    buf[39] = MAV_FRAME_BODY_NED;

    // 30..41: x, y, z (float, 3 * 4 = 12 bytes, unused)
    float zero = 0.0f;
    memcpy(&buf[40], &zero, 4);
    memcpy(&buf[44], &zero, 4);
    memcpy(&buf[48], &zero, 4);

    // 42..57: q[4] (float, 4 * 4 = 16 bytes: [1.0, 0.0, 0.0, 0.0])
    float q_w = 1.0f;
    memcpy(&buf[52], &q_w, 4);
    memcpy(&buf[56], &zero, 4);
    memcpy(&buf[60], &zero, 4);
    memcpy(&buf[64], &zero, 4);

    // 58: type (uint8_t, 1): LANDING_TARGET_TYPE_VISION_OTHER (1)
    buf[68] = LANDING_TARGET_TYPE_VISION_OTHER;
    // 59: position_valid (uint8_t, 1): 0 (angle-only, distance may be used)
    buf[69] = 0;

    // CRC Calculation (Header bytes 1..9 + Payload 60 bytes + CRC_EXTRA 200)
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 70; i++) {
        mavlink_crc_accumulate(buf[i], &crc);
    }
    mavlink_crc_accumulate(MAVLINK_MSG_ID_LANDING_TARGET_CRC_EXTRA, &crc);

    buf[70] = (uint8_t)(crc & 0xFF);
    buf[71] = (uint8_t)((crc >> 8) & 0xFF);

    return 72;
}
