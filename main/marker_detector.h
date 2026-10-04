#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_DETECTED_MARKERS 4

/**
 * @brief Detected fiducial marker / landing target information
 */
typedef struct {
    bool detected;
    uint8_t id;             // Marker ID (0: primary landing pad)
    float cx;               // Target center X (pixels)
    float cy;               // Target center Y (pixels)
    float width;            // Target width (pixels)
    float height;           // Target height (pixels)
    float angle_x;          // Horizontal angle from optical axis (radians)
    float angle_y;          // Vertical angle from optical axis (radians)
    float distance;         // Estimated distance in meters (-1 if unknown)
    int corners_x[4];       // 4 corner coordinates for overlay visualization
    int corners_y[4];
} marker_target_t;

/**
 * @brief Initialize the marker detector module.
 * 
 * @param fov_h_deg Horizontal Field of View of camera (typically 66.5 deg for Unit CamS3)
 * @param fov_v_deg Vertical Field of View of camera
 */
void marker_detector_init(float fov_h_deg, float fov_v_deg);

/**
 * @brief Process a grayscale frame to detect high-contrast fiducial markers / landing targets.
 * 
 * @param w Frame width (e.g. 160)
 * @param h Frame height (e.g. 120)
 * @param buf Grayscale pixel buffer (w * h bytes)
 * @param result Output pointer to receive detected target information
 * @return true if a valid target was detected, false otherwise
 */
bool marker_detector_process(uint16_t w, uint16_t h, const uint8_t *buf, marker_target_t *result);

#ifdef __cplusplus
}
#endif
