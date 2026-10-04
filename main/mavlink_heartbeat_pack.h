#pragma once

#include <stdint.h>
#include <string.h>

#define MAVLINK_V2_MAGIC 0xFD
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_HEARTBEAT_CRC_EXTRA 50

#define MAV_TYPE_ONBOARD_CONTROLLER 18
#define MAV_AUTOPILOT_INVALID 8
#define MAV_STATE_ACTIVE 4
#define MAV_COMP_ID_OPTICAL_FLOW 197

// X.25 / MCRF4XX CRC accumulator for MAVLink
static inline void mavlink_crc_accumulate_hb(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

/**
 * @brief Pack a HEARTBEAT message (MAVLink v2) for CamS3 Vision Hub
 *
 * @param buf Output buffer (must be at least 21 bytes)
 * @param seq Pointer to running sequence number (incremented automatically)
 * @return Total packet length in bytes (21 bytes)
 */
static inline uint16_t mavlink_pack_heartbeat_v2(uint8_t *buf, uint8_t *seq)
{
    // MAVLink v2 Header (10 bytes)
    buf[0] = MAVLINK_V2_MAGIC;                  // 0xFD
    buf[1] = MAVLINK_MSG_ID_HEARTBEAT_LEN;      // 9 bytes payload
    buf[2] = 0x00;                              // Incompat flags
    buf[3] = 0x00;                              // Compat flags
    buf[4] = (*seq)++;                          // Sequence number
    buf[5] = 1;                                 // System ID (matches vehicle sysid 1)
    buf[6] = MAV_COMP_ID_OPTICAL_FLOW;          // Component ID: 197
    buf[7] = 0x00;                              // Message ID: 0 (HEARTBEAT)
    buf[8] = 0x00;
    buf[9] = 0x00;

    // Payload (9 bytes, wire order ordered by type size in MAVLink spec)
    // 0..3: custom_mode (uint32_t, 4 bytes)
    uint32_t custom_mode = 0;
    memcpy(&buf[10], &custom_mode, 4);
    // 4: type (uint8_t, 1 byte) -> MAV_TYPE_ONBOARD_CONTROLLER (18)
    buf[14] = MAV_TYPE_ONBOARD_CONTROLLER;
    // 5: autopilot (uint8_t, 1 byte) -> MAV_AUTOPILOT_INVALID (8)
    buf[15] = MAV_AUTOPILOT_INVALID;
    // 6: base_mode (uint8_t, 1 byte) -> 0
    buf[16] = 0;
    // 7: system_status (uint8_t, 1 byte) -> MAV_STATE_ACTIVE (4)
    buf[17] = MAV_STATE_ACTIVE;
    // 8: mavlink_version (uint8_t, 1 byte) -> 3
    buf[18] = 3;

    // CRC Calculation (Header bytes 1..9 + Payload 9 bytes + CRC_EXTRA 50)
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 19; i++) {
        mavlink_crc_accumulate_hb(buf[i], &crc);
    }
    mavlink_crc_accumulate_hb(MAVLINK_MSG_ID_HEARTBEAT_CRC_EXTRA, &crc);

    buf[19] = (uint8_t)(crc & 0xFF);
    buf[20] = (uint8_t)((crc >> 8) & 0xFF);

    return 21;
}
