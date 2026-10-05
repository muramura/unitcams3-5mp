#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_ROUTER_UDP_PORT     14550
#define MAVLINK_ROUTER_MAX_CLIENTS  8
#define MAVLINK_ROUTER_BUF_SIZE     1400

typedef enum {
    MAV_ROUTE_DEST_FC  = (1 << 0),   // Route to StampFly Flight Controller (UART @ 2Mbps)
    MAV_ROUTE_DEST_GCS = (1 << 1),   // Route to all connected GCS / Companion endpoints (Wi-Fi UDP)
    MAV_ROUTE_DEST_ALL = (MAV_ROUTE_DEST_FC | MAV_ROUTE_DEST_GCS)
} mav_route_dest_t;

/**
 * @brief Initialize and start the MAVLink Multi-Endpoint Router on Core 1
 *
 * Implements full mavlink-routerd architecture:
 *  - Primary UART endpoint (StampFly FC via Grove G19/G20 @ 2,000,000 baud)
 *  - Multiple UDP endpoints (e.g. -e 192.168.4.2:14550, -e 192.168.4.3:14550, + dynamic GCS)
 *  - Bidirectional packet fan-out and cross-routing
 *  - Runs on Core 1 concurrently with camera streaming on Core 0
 */
esp_err_t mavlink_router_init(void);

/**
 * @brief Add or update a static UDP endpoint (equivalent to mavlink-routerd -e IP:PORT)
 * Static endpoints are permanently retained and never pruned by inactivity timeout.
 *
 * @param ip_str Destination IPv4 address string (e.g. "192.168.4.2")
 * @param port Destination UDP port (e.g. 14550)
 * @param is_static True if permanent static endpoint, false if dynamic
 * @return ESP_OK on success, ESP_ERR_NO_MEM if table full, or ESP_ERR_INVALID_ARG
 */
esp_err_t mavlink_router_add_endpoint(const char *ip_str, uint16_t port, bool is_static);

/**
 * @brief Thread-safe API for CamS3 internal subsystems
 *        to dispatch MAVLink packets simultaneously to FC, all connected GCS, or both.
 * @param packet Pointer to the MAVLink packet buffer
 * @param length Length of the packet in bytes
 * @param dest Destination mask (MAV_ROUTE_DEST_FC, MAV_ROUTE_DEST_GCS, or MAV_ROUTE_DEST_ALL)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if mutex busy, or error code
 */
esp_err_t mavlink_router_send_internal(const uint8_t *packet, uint16_t length, mav_route_dest_t dest);

/**
 * @brief Get the count of currently active GCS clients / endpoints
 */
int mavlink_router_get_active_client_count(void);

/**
 * @brief Router queue statistics structure
 */
typedef struct {
    uint32_t fc_tx_drops;
    uint32_t wifi_tx_drops;
    uint32_t bytes_to_fc;
    uint32_t bytes_to_wifi;
    uint32_t bytes_from_fc;
    uint32_t bytes_from_wifi;
} mavlink_router_stats_t;

/**
 * @brief Retrieve current queue drop counts and throughput statistics
 */
void mavlink_router_get_stats(mavlink_router_stats_t *stats);

#ifdef __cplusplus
}
#endif
