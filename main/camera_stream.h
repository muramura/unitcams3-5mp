#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define CAMERA_STREAM_PORT      80

/**
 * @brief Initialize and start the HTTP Video Streaming server on port 80.
 *        Endpoints:
 *        - /        : HTML Web HUD with live video
 *        - /stream  : Standard multipart/x-mixed-replace MJPEG video stream (Mission Planner / browser)
 *        - /capture : Single JPEG snapshot
 */
esp_err_t camera_stream_start(void);

/**
 * @brief Check if any streaming client is currently connected.
 *        Used by the optical flow capture loop to bypass frame copying and streaming overhead
 *        when no client is watching.
 */
bool camera_stream_has_client(void);

/**
 * @brief Push a newly captured JPEG frame from the optical flow capture loop into the
 *        ping-pong streaming buffer (zero-lock, non-blocking).
 */
void camera_stream_push_frame(const uint8_t *buf, size_t len);
