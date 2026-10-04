#pragma once

#include <stdint.h>
#include <string.h>

#define MAVLINK_V2_MAGIC 0xFD
#define MAVLINK_MSG_ID_OPTICAL_FLOW 100
#define MAVLINK_MSG_ID_OPTICAL_FLOW_LEN 34
#define MAVLINK_MSG_ID_OPTICAL_FLOW_CRC_EXTRA 175

// X.25 / MCRF4XX CRC accumulator for MAVLink
static inline void mavlink_crc_accumulate(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

/**
 * @brief Pack an OPTICAL_FLOW message (MAVLink v2)
 *
 * @param buf Output buffer (must be at least 46 bytes)
 * @param time_usec Timestamp in microseconds
 * @param sensor_id Sensor ID (e.g. 0)
 * @param flow_x Flow rate x (pixels or rad/s)
 * @param flow_y Flow rate y (pixels or rad/s)
 * @param flow_comp_m_x Flow in m/s x (angular compensated, or 0)
 * @param flow_comp_m_y Flow in m/s y (angular compensated, or 0)
 * @param quality Surface quality (0-255)
 * @param ground_distance Ground distance in meters (-1 if unknown)
 * @param flow_rate_x Angular flow rate x in rad/s (or 0)
 * @param flow_rate_y Angular flow rate y in rad/s (or 0)
 * @param seq Pointer to running sequence number (incremented automatically)
 * @return Total packet length in bytes (46 bytes)
 */
static inline uint16_t mavlink_pack_optical_flow_v2(
    uint8_t *buf,
    uint64_t time_usec,
    uint8_t sensor_id,
    int16_t flow_x,
    int16_t flow_y,
    float flow_comp_m_x,
    float flow_comp_m_y,
    uint8_t quality,
    float ground_distance,
    float flow_rate_x,
    float flow_rate_y,
    uint8_t *seq)
{
    // MAVLink v2 Header (10 bytes)
    buf[0] = MAVLINK_V2_MAGIC;                  // 0xFD
    buf[1] = MAVLINK_MSG_ID_OPTICAL_FLOW_LEN;   // 34 bytes payload
    buf[2] = 0x00;                              // Incompat flags
    buf[3] = 0x00;                              // Compat flags
    buf[4] = (*seq)++;                          // Sequence number
    buf[5] = 1;                                 // System ID (matches vehicle sysid or 1)
    buf[6] = 197;                               // Component ID: MAV_COMP_ID_OPTICAL_FLOW
    buf[7] = (uint8_t)(MAVLINK_MSG_ID_OPTICAL_FLOW & 0xFF);         // 100
    buf[8] = (uint8_t)((MAVLINK_MSG_ID_OPTICAL_FLOW >> 8) & 0xFF);  // 0
    buf[9] = (uint8_t)((MAVLINK_MSG_ID_OPTICAL_FLOW >> 16) & 0xFF); // 0

    // Payload (34 bytes, little-endian)
    // 0: time_usec (uint64_t, 8)
    memcpy(&buf[10], &time_usec, 8);
    // 8: flow_comp_m_x (float, 4)
    memcpy(&buf[18], &flow_comp_m_x, 4);
    // 12: flow_comp_m_y (float, 4)
    memcpy(&buf[22], &flow_comp_m_y, 4);
    // 16: ground_distance (float, 4)
    memcpy(&buf[26], &ground_distance, 4);
    // 20: flow_x (int16_t, 2)
    memcpy(&buf[30], &flow_x, 2);
    // 22: flow_y (int16_t, 2)
    memcpy(&buf[32], &flow_y, 2);
    // 24: sensor_id (uint8_t, 1)
    buf[34] = sensor_id;
    // 25: quality (uint8_t, 1)
    buf[35] = quality;
    // 26: flow_rate_x (float, 4)
    memcpy(&buf[36], &flow_rate_x, 4);
    // 30: flow_rate_y (float, 4)
    memcpy(&buf[40], &flow_rate_y, 4);

    // CRC Calculation (Header bytes 1..9 + Payload 34 bytes + CRC_EXTRA 175)
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 44; i++) {
        mavlink_crc_accumulate(buf[i], &crc);
    }
    mavlink_crc_accumulate(MAVLINK_MSG_ID_OPTICAL_FLOW_CRC_EXTRA, &crc);

    buf[44] = (uint8_t)(crc & 0xFF);
    buf[45] = (uint8_t)((crc >> 8) & 0xFF);

    return 46;
}
