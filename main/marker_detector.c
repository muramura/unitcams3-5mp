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

#define RAY_COUNT 36 // 36 rays = 10-degree angular resolution

void marker_detector_init(float fov_h_deg, float fov_v_deg)
{
    float fov_h_rad = fov_h_deg * ((float)M_PI / 180.0f);
    float fov_v_rad = fov_v_deg * ((float)M_PI / 180.0f);

    // Default assume 160x120 resolution
    s_cx0 = 160.0f / 2.0f;
    s_cy0 = 120.0f / 2.0f;
    s_fx = s_cx0 / tanf(fov_h_rad / 2.0f);
    s_fy = s_cy0 / tanf(fov_v_rad / 2.0f);

    ESP_LOGI(TAG, "Marker detector 6-DOF initialized: fx=%.1f, fy=%.1f, cx0=%.1f, cy0=%.1f",
             s_fx, s_fy, s_cx0, s_cy0);
}

/**
 * @brief Convert Euler angles (roll, pitch, yaw in radians) to normalized quaternion [w, x, y, z]
 */
static void euler_to_quaternion(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z

    // Normalize
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    } else {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    }
}

/**
 * @brief Fast fiducial marker detector with 6-DOF pose & orientation calculation
 */
bool marker_detector_process(uint16_t w, uint16_t h, const uint8_t *buf, float marker_size_m, marker_target_t *result)
{
    if (!buf || !result || w < 32 || h < 32) {
        return false;
    }

    memset(result, 0, sizeof(marker_target_t));

    if (marker_size_m <= 0.001f) {
        marker_size_m = 0.05f; // Default 50mm (0.05m)
    }

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

    // Require sufficient contrast
    if ((max_v - min_v) < 35) {
        return false;
    }

    uint8_t thresh = (uint8_t)((min_v + max_v) / 2);

    // 2. Coarse Candidate Search: Scan horizontal lines to find dark outer border
    int best_min_x = 0, best_max_x = 0;
    int best_min_y = 0, best_max_y = 0;
    int best_score = -1;

    int step_y = 4;
    int step_x = 2;
    int min_marker_size = 12;
    int max_marker_size = (int)(w * 0.90f);

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
                    int cand_w = dark_len;
                    int cand_h = cand_w;
                    int top = y - cand_h / 2;
                    int bottom = y + cand_h / 2;

                    if (top >= 2 && bottom < h - 2) {
                        int left = dark_start;
                        int right = x;

                        int center_x = (left + right) / 2;
                        int center_y = y;

                        uint8_t center_val = buf[center_y * w + center_x];
                        uint8_t top_edge   = buf[top * w + center_x];
                        uint8_t bot_edge   = buf[bottom * w + center_x];
                        uint8_t left_edge  = buf[center_y * w + left];
                        uint8_t right_edge = buf[center_y * w + right];

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

    if (best_score <= 25) {
        return false;
    }

    // 3. Fine Radial Ray-Casting from estimated center to extract 4 corners
    float coarse_cx = (float)(best_min_x + best_max_x) * 0.5f;
    float coarse_cy = (float)(best_min_y + best_max_y) * 0.5f;
    float coarse_r = (float)(best_max_x - best_min_x) * 0.5f;
    float max_search_r = coarse_r * 1.5f;
    if (max_search_r < 10.0f) max_search_r = 10.0f;

    typedef struct {
        float angle;
        float dist;
        float x;
        float y;
    } ray_hit_t;

    ray_hit_t rays[RAY_COUNT];

    for (int i = 0; i < RAY_COUNT; i++) {
        float ang = (float)i * (2.0f * (float)M_PI / (float)RAY_COUNT);
        float cos_a = cosf(ang);
        float sin_a = sinf(ang);

        rays[i].angle = ang;
        rays[i].dist = coarse_r;
        rays[i].x = coarse_cx + coarse_r * cos_a;
        rays[i].y = coarse_cy + coarse_r * sin_a;

        // Trace from center outward to find the transition from marker outer border (dark) to background (bright)
        float last_x = coarse_cx;
        float last_y = coarse_cy;
        bool in_dark_border = false;

        for (float r = 2.0f; r < max_search_r; r += 1.5f) {
            int px = (int)(coarse_cx + r * cos_a);
            int py = (int)(coarse_cy + r * sin_a);

            if (px < 1 || px >= w - 1 || py < 1 || py >= h - 1) {
                break;
            }

            uint8_t val = buf[py * w + px];
            if (val < thresh) {
                in_dark_border = true;
            } else if (in_dark_border && val >= thresh) {
                // Hit the outside boundary of the dark border!
                rays[i].dist = r;
                rays[i].x = (float)px;
                rays[i].y = (float)py;
                break;
            }
            last_x = (float)px;
            last_y = (float)py;
        }
    }

    // 4. Find 4 local maxima (peaks) of distance from center
    // For a square, corners have maximum radial distance (~1.414x side center distance)
    int peak_indices[RAY_COUNT];
    int peak_count = 0;

    for (int i = 0; i < RAY_COUNT; i++) {
        int prev = (i - 1 + RAY_COUNT) % RAY_COUNT;
        int next = (i + 1) % RAY_COUNT;

        if (rays[i].dist >= rays[prev].dist && rays[i].dist >= rays[next].dist && rays[i].dist > coarse_r * 0.7f) {
            peak_indices[peak_count++] = i;
        }
    }

    // Select the best 4 corners
    float corners_x[4];
    float corners_y[4];
    bool corners_found = false;

    if (peak_count >= 4) {
        // Sort peaks by distance descending
        for (int i = 0; i < peak_count - 1; i++) {
            for (int j = i + 1; j < peak_count; j++) {
                if (rays[peak_indices[j]].dist > rays[peak_indices[i]].dist) {
                    int tmp = peak_indices[i];
                    peak_indices[i] = peak_indices[j];
                    peak_indices[j] = tmp;
                }
            }
        }

        // Take top 4 peaks and sort by ray angle (clockwise order)
        int best4[4];
        for (int i = 0; i < 4; i++) best4[i] = peak_indices[i];
        
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 4; j++) {
                if (rays[best4[j]].angle < rays[best4[i]].angle) {
                    int tmp = best4[i];
                    best4[i] = best4[j];
                    best4[j] = tmp;
                }
            }
        }

        for (int i = 0; i < 4; i++) {
            corners_x[i] = rays[best4[i]].x;
            corners_y[i] = rays[best4[i]].y;
        }
        corners_found = true;
    }

    if (!corners_found) {
        // Fallback to bounding box corners
        corners_x[0] = (float)best_min_x; corners_y[0] = (float)best_min_y;
        corners_x[1] = (float)best_max_x; corners_y[1] = (float)best_min_y;
        corners_x[2] = (float)best_max_x; corners_y[2] = (float)best_max_y;
        corners_x[3] = (float)best_min_x; corners_y[3] = (float)best_max_y;
    }

    // 5. Align corners so P0 is top-left, P1 is top-right, P2 is bottom-right, P3 is bottom-left
    // Find corner with smallest Y (highest in image) or smallest X+Y
    int top_left_idx = 0;
    float min_dist_origin = 1e9f;
    for (int i = 0; i < 4; i++) {
        float d = corners_x[i] + corners_y[i]; // heuristic for top-left
        if (d < min_dist_origin) {
            min_dist_origin = d;
            top_left_idx = i;
        }
    }

    float sorted_x[4];
    float sorted_y[4];
    for (int i = 0; i < 4; i++) {
        sorted_x[i] = corners_x[(top_left_idx + i) % 4];
        sorted_y[i] = corners_y[(top_left_idx + i) % 4];
    }

    // Store ordered corners in result
    for (int i = 0; i < 4; i++) {
        result->corners_x[i] = (int)roundf(sorted_x[i]);
        result->corners_y[i] = (int)roundf(sorted_y[i]);
    }

    // 6. Geometry & Orientation Calculation
    // Center of marker (average of 4 corners)
    float cx = (sorted_x[0] + sorted_x[1] + sorted_x[2] + sorted_x[3]) * 0.25f;
    float cy = (sorted_y[0] + sorted_y[1] + sorted_y[2] + sorted_y[3]) * 0.25f;

    // Edge vectors
    float top_vx = sorted_x[1] - sorted_x[0];
    float top_vy = sorted_y[1] - sorted_y[0];
    float bot_vx = sorted_x[2] - sorted_x[3];
    float bot_vy = sorted_y[2] - sorted_y[3];
    float left_vx = sorted_x[3] - sorted_x[0];
    float left_vy = sorted_y[3] - sorted_y[0];
    float right_vx = sorted_x[2] - sorted_x[1];
    float right_vy = sorted_y[2] - sorted_y[1];

    float len_top = sqrtf(top_vx*top_vx + top_vy*top_vy);
    float len_bot = sqrtf(bot_vx*bot_vx + bot_vy*bot_vy);
    float len_left = sqrtf(left_vx*left_vx + left_vy*left_vy);
    float len_right = sqrtf(right_vx*right_vx + right_vy*right_vy);
    float len_avg = (len_top + len_bot + len_left + len_right) * 0.25f;

    if (len_avg < 4.0f) {
        return false;
    }

    // Horizontal reference vector (average of top and bottom edges)
    float h_vx = (top_vx + bot_vx) * 0.5f;
    float h_vy = (top_vy + bot_vy) * 0.5f;

    // Yaw angle (rotation of marker around optical axis in radians)
    // 0 rad = marker is aligned horizontally with image
    float yaw_rad = atan2f(h_vy, h_vx);
    float yaw_deg = yaw_rad * (180.0f / (float)M_PI);

    // Roll and Pitch tilt angles from perspective distortion
    float pitch_rad = atanf((len_top - len_bot) / len_avg);
    float roll_rad = atanf((len_right - len_left) / len_avg);
    float pitch_deg = pitch_rad * (180.0f / (float)M_PI);
    float roll_deg = roll_rad * (180.0f / (float)M_PI);

    // 7. 3D Position Estimation (Downward-facing camera)
    // Z distance from optical center along camera axis (meters)
    float f_avg = (s_fx + s_fy) * 0.5f;
    float z_dist = (marker_size_m * f_avg) / len_avg;

    // Camera frame coordinates
    // In downward camera configuration:
    // Image Y-up corresponds to Body Forward (+X)
    // Image X-right corresponds to Body Right (+Y)
    // Camera Z-down corresponds to Body Down (+Z)
    float body_x = -((cy - s_cy0) * z_dist) / s_fy; // Forward (m)
    float body_y =  ((cx - s_cx0) * z_dist) / s_fx; // Right (m)
    float body_z = z_dist;                          // Down (m)
    float total_distance = sqrtf(body_x*body_x + body_y*body_y + body_z*body_z);

    // Convert Euler angles to normalized quaternion [w, x, y, z]
    float q[4];
    euler_to_quaternion(roll_rad, pitch_rad, yaw_rad, q);

    // 8. Populate result structure
    result->detected = true;
    result->id = 0; // Primary landing target
    result->cx = cx;
    result->cy = cy;
    result->width = (len_top + len_bot) * 0.5f;
    result->height = (len_left + len_right) * 0.5f;
    result->angle_x = atanf((cx - s_cx0) / s_fx);
    result->angle_y = atanf((cy - s_cy0) / s_fy);
    result->distance = total_distance;

    result->yaw_deg = yaw_deg;
    result->roll_deg = roll_deg;
    result->pitch_deg = pitch_deg;
    result->pos_x = body_x;
    result->pos_y = body_y;
    result->pos_z = body_z;
    result->q[0] = q[0];
    result->q[1] = q[1];
    result->q[2] = q[2];
    result->q[3] = q[3];

    return true;
}
