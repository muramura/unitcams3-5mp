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

    // 2. Coarse Candidate Search: Multi-slice Horizontal Span Analysis
    // ArUco markers have distinct dark outer borders (left and right on scanlines)
    // with quiet zone (white/bright) outside and arbitrary bit patterns inside.
    typedef struct {
        int y;
        int cx;
        int w;
    } span_cand_t;

    #define MAX_CANDS 32
    span_cand_t cands[MAX_CANDS];
    int cand_count = 0;

    int min_size = 14;
    int max_size = (int)(w * 0.85f);
    int step_y = 3;

    for (int y = min_size / 2; y < h - min_size / 2; y += step_y) {
        int row = y * w;
        
        // Find dark segments on this row
        int seg_starts[16];
        int seg_ends[16];
        int seg_count = 0;
        bool in_dark = false;
        int start_x = 0;

        for (int x = 2; x < w - 2; x++) {
            bool dark = (buf[row + x] < thresh);
            if (dark && !in_dark) {
                in_dark = true;
                start_x = x;
            } else if (!dark && in_dark) {
                in_dark = false;
                if (seg_count < 16) {
                    seg_starts[seg_count] = start_x;
                    seg_ends[seg_count] = x - 1;
                    seg_count++;
                }
            }
        }
        if (in_dark && seg_count < 16) {
            seg_starts[seg_count] = start_x;
            seg_ends[seg_count] = w - 3;
            seg_count++;
        }

        // Test if segments form an ArUco outer border cross-section
        if (seg_count >= 2) {
            // Left border is first segment, right border is last segment
            int left_start = seg_starts[0];
            int left_end = seg_ends[0];
            int right_start = seg_starts[seg_count - 1];
            int right_end = seg_ends[seg_count - 1];

            int span_w = right_end - left_start;
            int t_left = left_end - left_start + 1;
            int t_right = right_end - right_start + 1;

            if (span_w >= min_size && span_w <= max_size &&
                t_left >= 1 && t_right >= 1 &&
                t_left < span_w / 2 && t_right < span_w / 2) {
                
                if (cand_count < MAX_CANDS) {
                    cands[cand_count].y = y;
                    cands[cand_count].cx = (left_start + right_end) / 2;
                    cands[cand_count].w = span_w;
                    cand_count++;
                }
            }
        } else if (seg_count == 1) {
            // Could be top or bottom solid border
            int span_w = seg_ends[0] - seg_starts[0];
            if (span_w >= min_size && span_w <= max_size) {
                if (cand_count < MAX_CANDS) {
                    cands[cand_count].y = y;
                    cands[cand_count].cx = (seg_starts[0] + seg_ends[0]) / 2;
                    cands[cand_count].w = span_w;
                    cand_count++;
                }
            }
        }
    }

    if (cand_count < 3) {
        return false;
    }

    // Sort cands by x to get robust median cx
    for (int i = 0; i < cand_count - 1; i++) {
        for (int j = i + 1; j < cand_count; j++) {
            if (cands[j].cx < cands[i].cx) {
                span_cand_t tmp = cands[i];
                cands[i] = cands[j];
                cands[j] = tmp;
            }
        }
    }
    float coarse_cx = (float)cands[cand_count / 2].cx;

    int min_cand_y = cands[0].y;
    int max_cand_y = cands[0].y;
    float sum_w = 0.0f;
    for (int i = 0; i < cand_count; i++) {
        if (cands[i].y < min_cand_y) min_cand_y = cands[i].y;
        if (cands[i].y > max_cand_y) max_cand_y = cands[i].y;
        sum_w += (float)cands[i].w;
    }
    float coarse_cy = (float)(min_cand_y + max_cand_y) * 0.5f;
    float coarse_w = sum_w / (float)cand_count;
    float coarse_h = (float)(max_cand_y - min_cand_y);

    // Aspect ratio check (allow tilt: 0.5 to 2.0)
    if (coarse_h < (float)min_size * 0.6f || coarse_w < (float)min_size) {
        return false;
    }
    float aspect = coarse_w / (coarse_h > 1.0f ? coarse_h : 1.0f);
    if (aspect < 0.50f || aspect > 2.0f) {
        return false;
    }

    float coarse_r = (coarse_w + coarse_h) * 0.25f;

    // 3. Fine Inward Radial Ray-Tracing from outside background toward center
    // Trace from bright background inward until hitting the dark outer border
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

        float max_r = coarse_r * 1.75f;
        float min_r = coarse_r * 0.35f;

        for (float r = max_r; r >= min_r; r -= 1.0f) {
            int px = (int)(coarse_cx + r * cos_a);
            int py = (int)(coarse_cy + r * sin_a);

            if (px < 1 || px >= w - 1 || py < 1 || py >= h - 1) {
                continue;
            }

            uint8_t val = buf[py * w + px];
            if (val < thresh) {
                // First dark pixel encountered coming from the outside background!
                rays[i].dist = r;
                rays[i].x = (float)px;
                rays[i].y = (float)py;
                break;
            }
        }
    }

    // 4. Find 4 local maxima (peaks) of distance from center
    // For a square, corners have maximum radial distance (~1.414x side center distance)
    int peak_indices[RAY_COUNT];
    int peak_count = 0;

    for (int i = 0; i < RAY_COUNT; i++) {
        int prev = (i - 1 + RAY_COUNT) % RAY_COUNT;
        int next = (i + 1) % RAY_COUNT;

        if (rays[i].dist >= rays[prev].dist && rays[i].dist >= rays[next].dist && rays[i].dist > coarse_r * 0.70f) {
            peak_indices[peak_count++] = i;
        }
    }

    // Select the best 4 corners
    float corners_x[4];
    float corners_y[4];

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
    } else {
        // Fallback: Pick highest radial peak in each of 4 quadrants (every 9 rays = 90 deg)
        int q_size = RAY_COUNT / 4;
        for (int q = 0; q < 4; q++) {
            int best_idx = q * q_size;
            float max_d = -1.0f;
            for (int j = 0; j < q_size; j++) {
                int idx = q * q_size + j;
                if (rays[idx].dist > max_d) {
                    max_d = rays[idx].dist;
                    best_idx = idx;
                }
            }
            corners_x[q] = rays[best_idx].x;
            corners_y[q] = rays[best_idx].y;
        }
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
