#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Optical flow metrics structure
 */
typedef struct {
    int16_t dx;          // Flow in pixels X
    int16_t dy;          // Flow in pixels Y
    uint8_t quality;     // Surface texture quality (0-255)
    uint32_t dt_us;      // Frame delta time in microseconds
    float fps;           // Calculated optical flow frame rate
} optical_flow_metrics_t;

/**
 * @brief Initialize the camera and optical flow processing pipeline.
 * Configures camera in high-speed grayscale mode (QQVGA 160x120 or HQVGA 240x176).
 * 
 * @return ESP_OK on success, or error code
 */
esp_err_t optical_flow_init(void);

/**
 * @brief Start the optical flow background task pinned to Core 0.
 * The task captures frames, computes ARPS block matching, and transmits
 * MAVLink #100 OPTICAL_FLOW packets to StampFly via UART (2Mbps).
 * 
 * @return ESP_OK on success, or error code
 */
esp_err_t optical_flow_start(void);

/**
 * @brief Retrieve the latest optical flow metrics.
 * 
 * @param metrics Pointer to destination struct
 */
void optical_flow_get_latest(optical_flow_metrics_t *metrics);

/**
 * @brief Start a standalone dummy test task sending fixed OPTICAL_FLOW (#100) messages.
 * Transmits fixed values (dx=123, dy=-456, quality=200, dist=0.5m) at 10Hz to both FC and GCS.
 * Used to verify the communication link before camera activation.
 * 
 * @return ESP_OK on success, or error code
 */
esp_err_t optical_flow_dummy_start(void);


#ifdef __cplusplus
}
#endif
