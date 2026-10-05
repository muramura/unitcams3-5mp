#include "optical_flow.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board_pins.h"
#include "mavlink_router.h"
#include "mavlink_optical_flow_pack.h"
#include "mavlink_landing_target_pack.h"
#include "jpeg_decoder.h"

static const char *TAG = "optical_flow";

// ==========================================
// ARPS Block Matching Configuration
// ==========================================
#define BM_PMAX        15   // Maximum search distance in pixels (±15 pixels)
#define BM_BSTEP        2   // Pixel subsampling step within the matching block
#define BM_WIDTH      160   // ARPS processing width
#define BM_HEIGHT     120   // ARPS processing height

// Global state & metrics
static optical_flow_metrics_t s_latest_metrics = {0};
static uint8_t s_mavlink_seq = 0;
static bool s_running = false;
static bool s_camera_ready = false;

// Decoupled Producer-Consumer Shared State
static portMUX_TYPE s_flow_lock = portMUX_INITIALIZER_UNLOCKED;
static int32_t s_accum_dx = 0;
static int32_t s_accum_dy = 0;
static uint8_t s_last_quality = 0;
static int64_t s_last_frame_us = 0;
static uint32_t s_new_frames_count = 0;

// ARPS algorithm helper buffers
static bool s_bm_done[2 * BM_PMAX + 1][2 * BM_PMAX + 1];

// Fast JPEG Decode & Grayscale Buffers (Allocated in PSRAM)
static uint16_t *s_rgb565_buf = NULL;
static uint8_t  *s_curr_gray = NULL;
static uint8_t  *s_prev_gray = NULL;
static bool      s_has_prev_frame = false;

// ==========================================
// Image Processing Core
// ==========================================

static inline uint32_t bm_SAD(uint16_t w, const uint8_t *buf1, const uint8_t *buf2,
                              uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh,
                              uint8_t bstep, int8_t dx, int8_t dy)
{
    uint32_t sum = 0;
    for (int y = by; y < by + bh; y += bstep) {
        int i1 = y * w + bx;
        int i2 = (y + dy) * w + bx + dx;
        for (int x = 0; x < bw; x += bstep) {
            sum += (uint32_t)abs((int16_t)buf1[i1] - (int16_t)buf2[i2]);
            i1 += bstep;
            i2 += bstep;
        }
    }
    return sum;
}

static uint8_t bm_calc_quality(uint16_t w, const uint8_t *buf, uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh, uint8_t bstep)
{
    int n = (bw / bstep) * (bh / bstep);
    if (n == 0) return 0;

    uint32_t sum = 0;
    for (int y = by; y < by + bh; y += bstep) {
        int row = y * w;
        for (int x = bx; x < bx + bw; x += bstep) {
            sum += buf[row + x];
        }
    }
    uint8_t mean = sum / n;

    uint32_t var_sum = 0;
    for (int y = by; y < by + bh; y += bstep) {
        int row = y * w;
        for (int x = bx; x < bx + bw; x += bstep) {
            var_sum += abs((int)buf[row + x] - (int)mean);
        }
    }
    uint32_t mav = var_sum / n;

    uint32_t q = mav * 10;
    if (q > 255) q = 255;
    return (uint8_t)q;
}

#define BM_CHECK(dx, dy) do { \
    if (-(p) <= (dx) && (dx) <= (p) && -(p) <= (dy) && (dy) <= (p) && !s_bm_done[(p) + (dx)][(p) + (dy)]) { \
        s_bm_done[(p) + (dx)][(p) + (dy)] = true; \
        uint32_t sum = bm_SAD(w, buf1, buf2, bx, by, bw, bh, bstep, (dx), (dy)); \
        if (min_sum > sum) { \
            min_sum = sum; \
            min_dx = (dx); \
            min_dy = (dy); \
        } \
    } \
} while(0)

static void bm_ARPS(uint16_t w, const uint8_t *buf1, const uint8_t *buf2,
                    uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh,
                    uint8_t bstep, int8_t p, int8_t *dx_io, int8_t *dy_io)
{
    memset(s_bm_done, 0, sizeof(s_bm_done));

    int8_t min_dx = 0;
    int8_t min_dy = 0;
    uint32_t min_sum = 0xFFFFFFFF;
    int8_t current_dx, current_dy;

    if (p > BM_PMAX) p = BM_PMAX;
    if (*dx_io < -p) *dx_io = -p;
    if (*dx_io > p) *dx_io = p;
    if (*dy_io < -p) *dy_io = -p;
    if (*dy_io > p) *dy_io = p;

    int S = abs(*dx_io);
    if (S < abs(*dy_io)) S = abs(*dy_io);
    if (S < 2) S = 2;

    // 1. Check previous frame motion vector
    BM_CHECK(*dx_io, *dy_io);

    // 2. Check center (0, 0)
    BM_CHECK(0, 0);

    // 3. 4-point "+" search at distance S
    BM_CHECK(+S, 0);
    BM_CHECK(-S, 0);
    BM_CHECK(0, +S);
    BM_CHECK(0, -S);

    // 4. Refinement
    do {
        current_dx = min_dx;
        current_dy = min_dy;

        BM_CHECK(current_dx + 1, current_dy);
        BM_CHECK(current_dx - 1, current_dy);
        BM_CHECK(current_dx, current_dy + 1);
        BM_CHECK(current_dx, current_dy - 1);
    } while (current_dx != min_dx || current_dy != min_dy);

    *dx_io = min_dx;
    *dy_io = min_dy;
}

// ======================================================================
// Task 1: Camera Capture & Motion Engine (Producer - Runs on Core 1)
// Captures native JPEG from PY260 5MP sensor, decodes 1/4 scale to 160x120,
// calculates ARPS block matching, and accumulates motion into shared state.
// Yields CPU (2ms) between frames to ensure Wi-Fi & router tasks run cleanly.
// ======================================================================

static void optical_flow_cam_task(void *pvParameters)
{
    ESP_LOGI(TAG, "PY260 Optical Flow Vision Engine started on Core %d", xPortGetCoreID());

    int8_t guess_dx = 0;
    int8_t guess_dy = 0;

    while (s_running) {
        // If camera hardware not ready, idle cleanly without thrashing
        if (!s_camera_ready) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Capture native JPEG frame from PY260 sensor (PSRAM buffer)
        camera_fb_t *fb_curr = esp_camera_fb_get();
        if (!fb_curr) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Fast hardware/DSP JPEG decode: VGA (640x480) -> 1/4 scale (160x120 RGB565)
        esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = fb_curr->buf,
            .indata_size = fb_curr->len,
            .outbuf = (uint8_t *)s_rgb565_buf,
            .outbuf_size = BM_WIDTH * BM_HEIGHT * sizeof(uint16_t),
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_1_4, // 640x480 -> 160x120
            .flags = { .swap_color_bytes = 0 },
        };
        esp_jpeg_image_output_t outimg;
        esp_err_t dec_res = esp_jpeg_decode(&jpeg_cfg, &outimg);

        // Immediately return camera frame buffer back to DMA queue
        esp_camera_fb_return(fb_curr);

        if (dec_res != ESP_OK || outimg.width != BM_WIDTH || outimg.height != BM_HEIGHT) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Fast RGB565 to 8-bit Grayscale conversion
        // Green channel (bits 5..10) provides 6 bits of high-contrast luminance
        for (int i = 0; i < BM_WIDTH * BM_HEIGHT; i++) {
            uint16_t p = s_rgb565_buf[i];
            s_curr_gray[i] = (uint8_t)(((p >> 5) & 0x3F) << 2);
        }

        int64_t now_us = esp_timer_get_time();

        // Run ARPS if we have a previous reference frame
        if (s_has_prev_frame) {
            uint16_t w = BM_WIDTH;
            uint16_t h = BM_HEIGHT;
            int p = BM_PMAX;
            int bstep = BM_BSTEP;
            int bx = p;
            int by = p;
            int bw = w - 2 * p;
            int bh = h - 2 * p;

            int8_t dx = guess_dx;
            int8_t dy = guess_dy;
            bm_ARPS(w, s_prev_gray, s_curr_gray, bx, by, bw, bh, bstep, p, &dx, &dy);
            guess_dx = dx;
            guess_dy = dy;

            uint8_t q = bm_calc_quality(w, s_curr_gray, bx, by, bw, bh, bstep);

            // Atomically update shared accumulator for TX task
            portENTER_CRITICAL(&s_flow_lock);
            s_accum_dx += dx;
            s_accum_dy += dy;
            s_last_quality = q;
            s_last_frame_us = now_us;
            s_new_frames_count++;
            portEXIT_CRITICAL(&s_flow_lock);
        }

        // Copy current to prev for next comparison
        memcpy(s_prev_gray, s_curr_gray, BM_WIDTH * BM_HEIGHT);
        s_has_prev_frame = true;

        // Yield CPU briefly to allow router tasks and other threads time to run
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    vTaskDelete(NULL);
}

// ======================================================================
// Task 2: Strictly Periodic MAVLink Telemetry Task (Consumer - Core 1)
// Runs at a crystal-clear, rock-solid 20.00 Hz (50.0ms) using vTaskDelayUntil.
// NEVER blocks on camera hardware or network!
// ======================================================================

static void optical_flow_tx_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Optical Flow Telemetry Broadcaster started on Core %d (Rock-Solid 20.0Hz)", xPortGetCoreID());

    uint8_t mav_packet[46];
    const TickType_t period_ticks = pdMS_TO_TICKS(50); // Exact 50.0 ms (20Hz)
    TickType_t last_wake_time = xTaskGetTickCount();

    int64_t last_stats_us = esp_timer_get_time();
    uint32_t packets_sent = 0;

    while (s_running) {
        vTaskDelayUntil(&last_wake_time, period_ticks);

        int64_t now_us = esp_timer_get_time();
        int16_t send_dx = 0;
        int16_t send_dy = 0;
        uint8_t send_quality = 0;

        // Atomically fetch and clear accumulated displacement
        portENTER_CRITICAL(&s_flow_lock);
        int32_t acc_x = s_accum_dx;
        int32_t acc_y = s_accum_dy;
        s_accum_dx = 0;
        s_accum_dy = 0;
        uint8_t q = s_last_quality;
        int64_t frame_age_us = now_us - s_last_frame_us;
        uint32_t frames_arrived = s_new_frames_count;
        s_new_frames_count = 0;
        portEXIT_CRITICAL(&s_flow_lock);

        // Condition evaluation
        if (s_camera_ready && (frame_age_us < 200000)) {
            // Camera is actively feeding frames: report real flow (accumulated dpix * 10)
            send_dx = (int16_t)(acc_x * 10);
            send_dy = (int16_t)(acc_y * 10);
            send_quality = (frames_arrived > 0) ? q : (q > 0 ? q : 0);
        } else {
            // Camera initializing, stalled, or NG: send_quality = 0 (NG / Hold state)
            send_dx = 0;
            send_dy = 0;
            send_quality = 0;
        }

        // Update public status metrics
        s_latest_metrics.dx = send_dx / 10;
        s_latest_metrics.dy = send_dy / 10;
        s_latest_metrics.quality = send_quality;
        s_latest_metrics.dt_us = 50000; // Exact 50ms interval

        // ALWAYS TRANSMIT MAVLINK #100 (Unconditional fixed-rate stream)
        uint16_t len = mavlink_pack_optical_flow_v2(
            mav_packet,
            (uint64_t)now_us,
            0,                      // sensor_id
            send_dx,                // flow_x (dpix * 10)
            send_dy,                // flow_y (dpix * 10)
            0.0f,                   // flow_comp_m_x
            0.0f,                   // flow_comp_m_y
            send_quality,           // quality: 0=NG, 1..255=Valid
            -1.0f,                  // ground_distance (-1: unknown)
            0.0f,                   // flow_rate_x
            0.0f,                   // flow_rate_y
            &s_mavlink_seq
        );

        mavlink_router_send_internal(mav_packet, len, MAV_ROUTE_DEST_ALL);
        packets_sent++;

        // Periodic diagnostics logging (every 5 seconds)
        if (now_us - last_stats_us >= 5000000) {
            float tx_rate = (float)packets_sent * 1000000.0f / (float)(now_us - last_stats_us);
            s_latest_metrics.fps = tx_rate;
            ESP_LOGI(TAG, "[OPTICAL FLOW] TX Rate: %.2f Hz | dx=%d, dy=%d | Quality=%u/255 %s",
                     tx_rate, send_dx, send_dy, send_quality,
                     (send_quality == 0) ? "(NG/HOLD)" : "(OK)");
            packets_sent = 0;
            last_stats_us = now_us;
        }
    }

    vTaskDelete(NULL);
}

// ==========================================
// Public API Functions
// ==========================================

esp_err_t optical_flow_init(void)
{
    ESP_LOGI(TAG, "Initializing PY260 camera for Optical Flow (VGA JPEG -> 1/4 160x120)...");

    // Allocate PSRAM buffers for JPEG decode and ARPS reference
    if (s_rgb565_buf == NULL) {
        s_rgb565_buf = (uint16_t *)heap_caps_malloc(BM_WIDTH * BM_HEIGHT * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_rgb565_buf) s_rgb565_buf = (uint16_t *)malloc(BM_WIDTH * BM_HEIGHT * sizeof(uint16_t));
    }
    if (s_curr_gray == NULL) {
        s_curr_gray = (uint8_t *)heap_caps_malloc(BM_WIDTH * BM_HEIGHT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_curr_gray) s_curr_gray = (uint8_t *)malloc(BM_WIDTH * BM_HEIGHT);
    }
    if (s_prev_gray == NULL) {
        s_prev_gray = (uint8_t *)heap_caps_malloc(BM_WIDTH * BM_HEIGHT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_prev_gray) s_prev_gray = (uint8_t *)malloc(BM_WIDTH * BM_HEIGHT);
    }

    if (!s_rgb565_buf || !s_curr_gray || !s_prev_gray) {
        ESP_LOGE(TAG, "Failed to allocate memory for JPEG decode & optical flow buffers!");
        return ESP_ERR_NO_MEM;
    }

    camera_config_t config = {
        .pin_pwdn     = CAMERA_PIN_PWDN,
        .pin_reset    = CAMERA_PIN_RESET,
        .pin_xclk     = CAMERA_PIN_XCLK,
        .pin_sccb_sda = CAMERA_PIN_SIOD,
        .pin_sccb_scl = CAMERA_PIN_SIOC,

        .pin_d7       = CAMERA_PIN_D7,
        .pin_d6       = CAMERA_PIN_D6,
        .pin_d5       = CAMERA_PIN_D5,
        .pin_d4       = CAMERA_PIN_D4,
        .pin_d3       = CAMERA_PIN_D3,
        .pin_d2       = CAMERA_PIN_D2,
        .pin_d1       = CAMERA_PIN_D1,
        .pin_d0       = CAMERA_PIN_D0,

        .pin_vsync    = CAMERA_PIN_VSYNC,
        .pin_href     = CAMERA_PIN_HREF,
        .pin_pclk     = CAMERA_PIN_PCLK,

        .xclk_freq_hz = BOARD_XCLK_FREQ_HZ,
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,        // PY260 native JPEG format
        .frame_size   = FRAMESIZE_VGA,         // 640x480 native
        .jpeg_quality = 14,
        .fb_count     = 2,                     // Double buffer in PSRAM
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PY260 Camera init failed: 0x%x! Telemetry will broadcast in NG mode.", err);
        s_camera_ready = false;
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 0);
        ESP_LOGI(TAG, "PY260 Camera successfully initialized! (PID: 0x%04x)", s->id.PID);
    }

    s_camera_ready = true;
    return ESP_OK;
}

esp_err_t optical_flow_start(void)
{
    if (s_running) {
        return ESP_OK;
    }
    s_running = true;

    // Pin both optical flow tasks to Core 1 at cooperative priorities.
    // Core 0 is 100% RESERVED FOR WI-FI SoftAP & System Interrupts!
    // 1. Camera Vision Engine (Producer) on Core 1, priority 4 (8KB stack for esp_jpeg decode)
    xTaskCreatePinnedToCore(
        optical_flow_cam_task,
        "opt_flow_cam",
        8192,
        NULL,
        4,
        NULL,
        1   // Core 1
    );

    // 2. Periodic MAVLink Broadcaster (Consumer) on Core 1, priority 5
    BaseType_t res = xTaskCreatePinnedToCore(
        optical_flow_tx_task,
        "opt_flow_tx",
        3072,
        NULL,
        5,
        NULL,
        1   // Core 1
    );

    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}

void optical_flow_get_latest(optical_flow_metrics_t *metrics)
{
    if (metrics != NULL) {
        *metrics = s_latest_metrics;
    }
}

// ==========================================
// Standalone Dummy Test Task (MAVLink #100)
// ==========================================

static void optical_flow_dummy_task(void *pvParameters)
{
    ESP_LOGI(TAG, "==========================================================");
    ESP_LOGI(TAG, "[DUMMY TEST] Sender started (Core 1)");
    ESP_LOGI(TAG, "  - #100 OPTICAL_FLOW   @ 10Hz: flow_x=123, flow_y=-456, qual=200, dist=0.5m");
    ESP_LOGI(TAG, "  - #149 LANDING_TARGET @  5Hz: angle_x=0.12, angle_y=-0.34, dist=1.5m, id=0");
    ESP_LOGI(TAG, "  Target: StampFly FC (UART 2Mbps) + GCS (UDP 14550)");
    ESP_LOGI(TAG, "==========================================================");

    uint8_t seq = 0;
    uint8_t flow_packet[64];
    uint8_t target_packet[80];
    uint32_t count = 0;

    const int16_t dummy_flow_x = 123;
    const int16_t dummy_flow_y = -456;
    const uint8_t dummy_quality = 200;
    const float dummy_flow_dist = 0.5f;

    const uint8_t dummy_target_id = 0;
    const float dummy_angle_x = 0.12f;
    const float dummy_angle_y = -0.34f;
    const float dummy_target_dist = 1.50f;
    const float dummy_size_x = 0.05f;
    const float dummy_size_y = 0.05f;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
        count++;

        uint64_t now_us = (uint64_t)esp_timer_get_time();

        // 1. Send OPTICAL_FLOW (#100) @ 10Hz
        uint16_t flow_len = mavlink_pack_optical_flow_v2(
            flow_packet,
            now_us,
            0,
            dummy_flow_x,
            dummy_flow_y,
            0.0f,
            0.0f,
            dummy_quality,
            dummy_flow_dist,
            0.0f,
            0.0f,
            &seq
        );
        mavlink_router_send_internal(flow_packet, flow_len, MAV_ROUTE_DEST_ALL);

        // 2. Send LANDING_TARGET (#149) @ 5Hz
        if (count % 2 == 0) {
            uint16_t target_len = mavlink_pack_landing_target_v2(
                target_packet,
                now_us,
                dummy_target_id,
                dummy_angle_x,
                dummy_angle_y,
                dummy_target_dist,
                dummy_size_x,
                dummy_size_y,
                &seq
            );
            mavlink_router_send_internal(target_packet, target_len, MAV_ROUTE_DEST_ALL);
        }

        if (count % 30 == 0) {
            ESP_LOGI(TAG, "[DUMMY TEST] Active: #100 Flow @ 10Hz | #149 Target @ 5Hz (Count: %u)", (unsigned int)count);
        }
    }
}

esp_err_t optical_flow_dummy_start(void)
{
    BaseType_t res = xTaskCreatePinnedToCore(
        optical_flow_dummy_task,
        "opt_flow_dummy",
        3072,
        NULL,
        5,
        NULL,
        1   // Core 1
    );
    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}
