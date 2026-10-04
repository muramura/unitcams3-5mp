#include "marker_detector.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "marker_detector";

static float s_fx = 120.0f; // Focal length in pixels (X)
static float s_fy = 120.0f; // Focal length in pixels (Y)
static float s_cx0 = 80.0f; // Optical center X
static float s_cy0 = 60.0f; // Optical center Y

void marker_detector_init(float fov_h_deg, float fov_v_deg)
{
    float fov_h_rad = fov_h_deg * (M_PI / 180.0f);
    float fov_v_rad = fov_v_deg * (M_PI / 180.0f);

    // Default assume 160x120 resolution
    s_cx0 = 160.0f / 2.0f;
    s_cy0 = 120.0f / 2.0f;
    s_fx = s_cx0 / tanf(fov_h_rad / 2.0f);
    s_fy = s_cy0 / tanf(fov_v_rad / 2.0f);

    ESP_LOGI(TAG, "Marker detector initialized: fx=%.1f, fy=%.1f, cx0=%.1f, cy0=%.1f",
             s_fx, s_fy, s_cx0, s_cy0);
}

/**
 * @brief Fast single-pass candidate bounding box detection for square fiducial markers.
 */
bool marker_detector_process(uint16_t w, uint16_t h, const uint8_t *buf, marker_target_t *result)
{
    if (!buf || !result || w < 32 || h < 32) {
        return false;
    }

    memset(result, 0, sizeof(marker_target_t));

    // 1. Calculate fast scene brightness range (sample every 4th pixel)
    uint32_t sum = 0;
    uint8_t min_v = 255;
    uint8_t max_v = 0;
    int samples = 0;

    for (int y = 4; y < h - 4; y += 4) {
        int row = y * w;
        for (int x = 4; x < w - 4; x += 4) {
            uint8_t v = buf[row + x];
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
            samples++;
        }
    }

    // Require sufficient contrast for marker detection
    if ((max_v - min_v) < 40) {
        return false;
    }

    uint8_t thresh = (uint8_t)((min_v + max_v) / 2);

    // 2. Scan for dark quad/border bounding candidates
    // A landing marker / ArUco marker has a distinct dark outer border with bright interior
    int best_min_x = 0, best_max_x = 0;
    int best_min_y = 0, best_max_y = 0;
    int best_score = -1;

    // Scan horizontal lines to find dark segments
    int step_y = 4;
    int step_x = 2;

    int min_marker_size = 14;
    int max_marker_size = (int)(w * 0.85f);

    for (int y = min_marker_size; y < h - min_marker_size; y += step_y) {
        int row = y * w;
        int dark_start = -1;

        for (int x = 2; x < w - 2; x += step_x) {
            bool is_dark = (buf[row + x] < thresh);

            if (is_dark && dark_start < 0) {
                dark_start = x;
            } else if (!is_dark && dark_start >= 0) {
                int dark_len = x - dark_start;
                if (dark_len >= min_marker_size && dark_len <= max_marker_size) {
                    // Test if this candidate segment belongs to a square marker box
                    int cand_w = dark_len;
                    int cand_h = cand_w; // Expect square aspect ratio
                    int top = y - cand_h / 2;
                    int bottom = y + cand_h / 2;

                    if (top >= 2 && bottom < h - 2) {
                        int left = dark_start;
                        int right = x;

                        // Check outer border vs inner center contrast
                        int center_x = (left + right) / 2;
                        int center_y = y;

                        uint8_t center_val = buf[center_y * w + center_x];
                        uint8_t top_edge   = buf[top * w + center_x];
                        uint8_t bot_edge   = buf[bottom * w + center_x];
                        uint8_t left_edge  = buf[center_y * w + left];
                        uint8_t right_edge = buf[center_y * w + right];

                        // Target condition: dark outer border + brighter center (or ArUco inner pattern)
                        if (top_edge < thresh && bot_edge < thresh &&
                            left_edge < thresh && right_edge < thresh &&
                            center_val > thresh) {
                            
                            int score = (int)center_val - (int)((top_edge + bot_edge + left_edge + right_edge) / 4);
                            if (score > best_score) {
                                best_score = score;
                                best_min_x = left;
                                best_max_x = right;
                                best_min_y = top;
                                best_max_y = bottom;
                            }
                        }
                    }
                }
                dark_start = -1;
            }
        }
    }

    if (best_score > 30) {
        // Target found!
        result->detected = true;
        result->id = 0; // Default Landing Pad ID 0
        result->cx = (float)(best_min_x + best_max_x) / 2.0f;
        result->cy = (float)(best_min_y + best_max_y) / 2.0f;
        result->width = (float)(best_max_x - best_min_x);
        result->height = (float)(best_max_y - best_min_y);

        // Store 4 corners for overlay visualization
        result->corners_x[0] = best_min_x; result->corners_y[0] = best_min_y;
        result->corners_x[1] = best_max_x; result->corners_y[1] = best_min_y;
        result->corners_x[2] = best_max_x; result->corners_y[2] = best_max_y;
        result->corners_x[3] = best_min_x; result->corners_y[3] = best_max_y;

        // Calculate line-of-sight angles in radians (positive X = right, positive Y = down)
        result->angle_x = atanf((result->cx - s_cx0) / s_fx);
        result->angle_y = atanf((result->cy - s_cy0) / s_fy);

        // Estimated distance: assumed physical landing pad width = 0.15m (15cm)
        // distance = (real_width * fx) / pixel_width
        if (result->width > 2.0f) {
            result->distance = (0.15f * s_fx) / result->width;
        } else {
            result->distance = -1.0f;
        }

        return true;
    }

    return false;
}
