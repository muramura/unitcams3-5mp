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

    s_cx0 = 160.0f / 2.0f;
    s_cy0 = 120.0f / 2.0f;
    s_fx = s_cx0 / tanf(fov_h_rad / 2.0f);
    s_fy = s_cy0 / tanf(fov_v_rad / 2.0f);

    ESP_LOGI(TAG, "Marker detector initialized: fx=%.1f, fy=%.1f, cx0=%.1f, cy0=%.1f",
             s_fx, s_fy, s_cx0, s_cy0);
}

/**
 * @brief Helper: Convert Euler angles (roll, pitch, yaw in radians) to normalized quaternion [w, x, y, z]
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
 * @brief Robust ArUco / landing marker detector with 6-DOF pose & orientation calculation
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

    // 1. Calculate dynamic scene brightness range
    uint8_t min_v = 255;
    uint8_t max_v = 0;

    for (int y = 4; y < h - 4; y += 4) {
        int row = y * w;
        for (int x = 4; x < w - 4; x += 4) {
            uint8_t v = buf[row + x];
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
    }

    // Require sufficient contrast
    if ((max_v - min_v) < 35) {
        return false;
    }

    uint8_t thresh = (uint8_t)((min_v + max_v) / 2);

    // 2. Candidate Row Scanning
    // An ArUco marker has a distinct dark bounding region surrounded by a bright quiet zone.
    // We ignore segments that touch image borders (which are typically desk or background darkness).
    #define MAX_ROW_CANDS 64
    typedef struct {
        int y;
        int left;
        int right;
        int w;
    } row_cand_t;

    row_cand_t row_cands[MAX_ROW_CANDS];
    int cand_count = 0;

    int min_size = 14;
    int max_size = (int)(w * 0.85f);
    int step_y = 2;

    for (int y = min_size / 2; y < h - min_size / 2; y += step_y) {
        int row = y * w;
        
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

        // Filter: Keep only dark segments that do NOT touch screen boundaries
        int valid_starts[16];
        int valid_ends[16];
        int valid_count = 0;
        for (int i = 0; i < seg_count; i++) {
            if (seg_starts[i] > 3 && seg_ends[i] < w - 4) {
                valid_starts[valid_count] = seg_starts[i];
                valid_ends[valid_count] = seg_ends[i];
                valid_count++;
            }
        }

        if (valid_count >= 1) {
            int span_left = valid_starts[0];
            int span_right = valid_ends[valid_count - 1];
            int span_w = span_right - span_left + 1;

            // Verify bright quiet zone on immediate left and right
            if (span_left >= 2 && span_right < w - 2) {
                if (buf[row + span_left - 1] >= thresh && buf[row + span_right + 1] >= thresh) {
                    if (span_w >= min_size && span_w <= max_size) {
                        if (cand_count < MAX_ROW_CANDS) {
                            row_cands[cand_count].y = y;
                            row_cands[cand_count].left = span_left;
                            row_cands[cand_count].right = span_right;
                            row_cands[cand_count].w = span_w;
                            cand_count++;
                        }
                    }
                }
            }
        }
    }

    if (cand_count < 4) {
        return false;
    }

    // 3. Determine marker bounding box & center
    int min_x = w, max_x = 0;
    int min_y = h, max_y = 0;
    for (int i = 0; i < cand_count; i++) {
        if (row_cands[i].left < min_x) min_x = row_cands[i].left;
        if (row_cands[i].right > max_x) max_x = row_cands[i].right;
        if (row_cands[i].y < min_y) min_y = row_cands[i].y;
        if (row_cands[i].y > max_y) max_y = row_cands[i].y;
    }

    float box_w = (float)(max_x - min_x + 1);
    float box_h = (float)(max_y - min_y + 1);

    if (box_w < (float)min_size || box_h < (float)min_size) {
        return false;
    }

    float aspect = box_w / (box_h > 1.0f ? box_h : 1.0f);
    if (aspect < 0.45f || aspect > 2.2f) {
        return false;
    }

    float coarse_cx = (float)(min_x + max_x) * 0.5f;
    float coarse_cy = (float)(min_y + max_y) * 0.5f;
    float coarse_r = (box_w + box_h) * 0.25f;

    // 4. Corner Extraction: Outward Ray-Casting from center to outer quiet-zone boundary
    #define RAY_COUNT 36
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

        // Trace outward from coarse_r * 0.5 to coarse_r * 1.8 to find transition from dark border to bright quiet zone
        float max_search_r = coarse_r * 1.75f;
        float min_search_r = coarse_r * 0.50f;

        for (float r = min_search_r; r <= max_search_r; r += 1.0f) {
            int px = (int)(coarse_cx + r * cos_a);
            int py = (int)(coarse_cy + r * sin_a);
            int px_out = (int)(coarse_cx + (r + 2.0f) * cos_a);
            int py_out = (int)(coarse_cy + (r + 2.0f) * sin_a);

            if (px < 1 || px >= w - 1 || py < 1 || py >= h - 1 ||
                px_out < 0 || px_out >= w || py_out < 0 || py_out >= h) {
                break;
            }

            uint8_t val_cur = buf[py * w + px];
            uint8_t val_out = buf[py_out * w + px_out];

            // Dark border transition to bright quiet zone
            if (val_cur < thresh && val_out >= thresh) {
                rays[i].dist = r;
                rays[i].x = (float)px;
                rays[i].y = (float)py;
                break;
            }
        }
    }

    // Find 4 local maxima (peaks) of radial distance
    int peak_indices[RAY_COUNT];
    int peak_count = 0;

    for (int i = 0; i < RAY_COUNT; i++) {
        int prev = (i - 1 + RAY_COUNT) % RAY_COUNT;
        int next = (i + 1) % RAY_COUNT;

        if (rays[i].dist >= rays[prev].dist && rays[i].dist >= rays[next].dist && rays[i].dist > coarse_r * 0.75f) {
            peak_indices[peak_count++] = i;
        }
    }

    // Select 4 corners using Non-Maximum Suppression (>= 5 rays = 50 deg apart)
    float corners_x[4];
    float corners_y[4];
    int chosen[4];
    int chosen_count = 0;

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

    for (int i = 0; i < peak_count && chosen_count < 4; i++) {
        int cand_idx = peak_indices[i];
        bool too_close = false;
        for (int c = 0; c < chosen_count; c++) {
            int diff = abs(cand_idx - chosen[c]);
            if (diff > RAY_COUNT / 2) {
                diff = RAY_COUNT - diff;
            }
            if (diff < 5) { // < 50 deg
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            chosen[chosen_count++] = cand_idx;
        }
    }

    if (chosen_count == 4) {
        // Sort chosen corners by ray angle (clockwise order)
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 4; j++) {
                if (rays[chosen[j]].angle < rays[chosen[i]].angle) {
                    int tmp = chosen[i];
                    chosen[i] = chosen[j];
                    chosen[j] = tmp;
                }
            }
        }
        for (int i = 0; i < 4; i++) {
            corners_x[i] = rays[chosen[i]].x;
            corners_y[i] = rays[chosen[i]].y;
        }
    } else {
        // Robust Fallback: Use Bounding Box 4 corners directly
        corners_x[0] = (float)min_x; corners_y[0] = (float)min_y; // Top-Left
        corners_x[1] = (float)max_x; corners_y[1] = (float)min_y; // Top-Right
        corners_x[2] = (float)max_x; corners_y[2] = (float)max_y; // Bottom-Right
        corners_x[3] = (float)min_x; corners_y[3] = (float)max_y; // Bottom-Left
    }

    // 5. Order corners: P0=Top-Left, P1=Top-Right, P2=Bottom-Right, P3=Bottom-Left
    int top_left_idx = 0;
    float min_dist_origin = 1e9f;
    for (int i = 0; i < 4; i++) {
        float d = corners_x[i] + corners_y[i];
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

    for (int i = 0; i < 4; i++) {
        result->corners_x[i] = (int)roundf(sorted_x[i]);
        result->corners_y[i] = (int)roundf(sorted_y[i]);
    }

    // 6. Geometry & Orientation Calculation
    float cx = (sorted_x[0] + sorted_x[1] + sorted_x[2] + sorted_x[3]) * 0.25f;
    float cy = (sorted_y[0] + sorted_y[1] + sorted_y[2] + sorted_y[3]) * 0.25f;

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

    // Horizontal reference vector
    float h_vx = (top_vx + bot_vx) * 0.5f;
    float h_vy = (top_vy + bot_vy) * 0.5f;

    float yaw_rad = atan2f(h_vy, h_vx);
    float yaw_deg = yaw_rad * (180.0f / (float)M_PI);

    float pitch_rad = atanf((len_top - len_bot) / len_avg);
    float roll_rad = atanf((len_right - len_left) / len_avg);
    float pitch_deg = pitch_rad * (180.0f / (float)M_PI);
    float roll_deg = roll_rad * (180.0f / (float)M_PI);

    // 7. 3D Position Estimation (FRD coordinates)
    float f_avg = (s_fx + s_fy) * 0.5f;
    float z_dist = (marker_size_m * f_avg) / len_avg;

    float body_x = -((cy - s_cy0) * z_dist) / s_fy; // Forward (m)
    float body_y =  ((cx - s_cx0) * z_dist) / s_fx; // Right (m)
    float body_z = z_dist;                          // Down (m)
    float total_distance = sqrtf(body_x*body_x + body_y*body_y + body_z*body_z);

    float q[4];
    euler_to_quaternion(roll_rad, pitch_rad, yaw_rad, q);

    // 8. Populate result
    result->detected = true;
    result->id = 0;
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
