#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_DETECTED_MARKERS 4

/**
 * @brief Detected fiducial marker / landing target information with 6-DOF Pose
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
    float distance;         // Estimated 3D Euclidean distance in meters (-1 if unknown)
    int corners_x[4];       // 4 corner coordinates in image (P0: top-left, P1: top-right, P2: bottom-right, P3: bottom-left)
    int corners_y[4];

    // 6-DOF Orientation and 3D Position
    float yaw_deg;          // Heading / Yaw angle in degrees (-180 to +180 deg)
    float roll_deg;         // Roll tilt in degrees
    float pitch_deg;        // Pitch tilt in degrees
    float pos_x;            // Body Forward position (m) in MAV_FRAME_BODY_FRD
    float pos_y;            // Body Right position (m) in MAV_FRAME_BODY_FRD
    float pos_z;            // Body Down position (m) in MAV_FRAME_BODY_FRD
    float q[4];             // Target orientation quaternion [w, x, y, z] for ArduPilot
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
 * @param marker_size_m Physical size of marker side in meters (e.g. 0.05m = 50mm)
 * @param result Output pointer to receive detected target information
 * @return true if a valid target was detected, false otherwise
 */
bool marker_detector_process(uint16_t w, uint16_t h, const uint8_t *buf, float marker_size_m, marker_target_t *result);

/**
 * @brief Update latest detection result in thread-safe storage (called by vision loop)
 */
void marker_detector_update(const marker_target_t *target);

/**
 * @brief Get copy of latest detection result (thread-safe, called by web / telemetry)
 */
void marker_detector_get_latest(marker_target_t *out);

#ifdef __cplusplus
}
#endif
