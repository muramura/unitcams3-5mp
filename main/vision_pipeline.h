#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "optical_flow.h"
#include "marker_detector.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Master Vision Pipeline System Status
 */
typedef struct {
    optical_flow_metrics_t flow;
    marker_target_t target;
    uint32_t frame_count;
    float pipeline_fps;
} vision_pipeline_status_t;

/**
 * @brief Initialize all camera and vision sub-pipelines (Optical Flow + ArUco/Marker Detector).
 * Configures camera in high-speed Grayscale mode (160x120) with triple buffering.
 * 
 * @return ESP_OK on success
 */
esp_err_t vision_pipeline_init(void);

/**
 * @brief Start the master vision dispatcher task pinned to Core 0.
 * Concurrently processes Optical Flow (30 FPS) and Marker Detection (6 FPS),
 * transmitting MAVLink #100 and #149 to StampFly at 2Mbps.
 * 
 * @return ESP_OK on success
 */
esp_err_t vision_pipeline_start(void);

/**
 * @brief Get current snapshot of the vision pipeline status.
 * 
 * @param status Pointer to destination status struct
 */
void vision_pipeline_get_status(vision_pipeline_status_t *status);

/**
 * @brief Hook for HTTP server to fetch latest frame with OSD overlay in JPEG format.
 * 
 * @param out_jpg Pointer to receive pointer to allocated JPEG buffer (must be free'd by caller)
 * @param out_len Pointer to receive length of JPEG buffer
 * @return ESP_OK on success, or ESP_FAIL
 */
esp_err_t vision_pipeline_get_overlay_jpg(uint8_t **out_jpg, size_t *out_len);

#ifdef __cplusplus
}
#endif
