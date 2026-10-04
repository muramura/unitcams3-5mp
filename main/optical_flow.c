#include "optical_flow.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "board_pins.h"
#include "mavlink_optical_flow_pack.h"
#include "mavlink_heartbeat_pack.h"
#include "mavlink_router.h"

static const char *TAG = "optical_flow";

// ==========================================
// Block Matching Configuration
// ==========================================
#define BM_PMAX        15   // Maximum search distance in pixels (±15 pixels)
#define BM_BSTEP        2   // Pixel subsampling step within the matching block
#define BM_FRAME_SIZE   FRAMESIZE_QQVGA  // 160x120 grayscale (optimal balance of speed and field of view)

// Global state & metrics
static optical_flow_metrics_t s_latest_metrics = {0};
static uint8_t s_mavlink_seq = 0;
static bool s_running = false;

// ARPS algorithm helper buffers
static bool s_bm_done[2 * BM_PMAX + 1][2 * BM_PMAX + 1];

// ==========================================
// Image Processing Core
// ==========================================

/**
 * @brief Sum of Absolute Differences (SAD) between buf1 (reference) and buf2 (current frame).
 */
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

/**
 * @brief Mean Absolute Variance: measures texture/contrast of the ground to compute surface quality.
 * Returns quality score in range 0 - 255.
 */
static uint8_t bm_calc_quality(uint16_t w, const uint8_t *buf, uint16_t bx, uint16_t by, uint8_t bw, uint8_t bh, uint8_t bstep)
{
    int n = (bw / bstep) * (bh / bstep);
    if (n == 0) return 0;

    // 1. Calculate average brightness
    uint32_t sum = 0;
    for (int j = by * w + bx; j < (by + bh) * w; j += bstep * w) {
        for (int i = j; i < j + bw; i += bstep) {
            sum += buf[i];
        }
    }
    uint8_t avg = (uint8_t)((sum + n / 2) / n);

    // 2. Calculate variance from average
    uint32_t diff_sum = 0;
    for (int j = by * w + bx; j < (by + bh) * w; j += bstep * w) {
        for (int i = j; i < j + bw; i += bstep) {
            diff_sum += (uint32_t)abs((int16_t)buf[i] - (int16_t)avg);
        }
    }
    uint32_t mav = (diff_sum + n / 2) / n;

    // Scale MAV to 0-255 range (typical ground texture gives MAV 5-40)
    uint32_t q = mav * 8;
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

/**
 * @brief Adaptive Rood Pattern Search (ARPS) Block Matching.
 * Fast, highly efficient motion estimation (< 3ms on ESP32-S3).
 */
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

    // Initial search radius S from previous frame estimate
    int S = abs(*dx_io);
    if (S < abs(*dy_io)) S = abs(*dy_io);
    if (S < 2) S = 2;

    // 1. Check previous frame motion vector
    BM_CHECK(*dx_io, *dy_io);

    // 2. Check center (0, 0)
    BM_CHECK(0, 0);

    // 3. 4-point '+' search at distance S
    BM_CHECK(+S, 0);
    BM_CHECK(-S, 0);
    BM_CHECK(0, +S);
    BM_CHECK(0, -S);

    // 4. Refinement: unit step search around current minimum until center is best
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

// ==========================================
// Optical Flow Task Loop
// ==========================================

static void optical_flow_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Optical Flow task running on Core %d", xPortGetCoreID());

    camera_fb_t *fb_last = esp_camera_fb_get();
    if (!fb_last) {
        ESP_LOGE(TAG, "Failed to get initial frame buffer!");
        vTaskDelete(NULL);
        return;
    }

    int8_t guess_dx = 0;
    int8_t guess_dy = 0;
    int64_t last_time_us = esp_timer_get_time();
    int64_t last_stats_us = last_time_us;
    uint32_t frame_count = 0;
    uint8_t mav_packet[46];

    while (s_running) {
        camera_fb_t *fb_curr = esp_camera_fb_get();
        if (!fb_curr) {
            ESP_LOGW(TAG, "Camera frame capture dropped");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        uint32_t dt_us = (uint32_t)(now_us - last_time_us);
        last_time_us = now_us;

        uint16_t w = fb_curr->width;
        uint16_t h = fb_curr->height;
        int p = BM_PMAX;
        int bstep = BM_BSTEP;
        int bx = p;
        int by = p;
        int bw = w - 2 * p;
        int bh = h - 2 * p;

        // Execute fast ARPS block matching
        int8_t dx = guess_dx;
        int8_t dy = guess_dy;
        bm_ARPS(w, fb_last->buf, fb_curr->buf, bx, by, bw, bh, bstep, p, &dx, &dy);
        guess_dx = dx;
        guess_dy = dy;

        // Calculate surface texture quality
        uint8_t quality = bm_calc_quality(w, fb_curr->buf, bx, by, bw, bh, bstep);

        // Update public metrics
        s_latest_metrics.dx = dx;
        s_latest_metrics.dy = dy;
        s_latest_metrics.quality = quality;
        s_latest_metrics.dt_us = dt_us;

        // Pack MAVLink v2 OPTICAL_FLOW (#100) packet
        // flow_x, flow_y: pixel flow * 10 (standard ArduPilot integer format)
        uint16_t len = mavlink_pack_optical_flow_v2(
            mav_packet,
            (uint64_t)now_us,
            0,                      // sensor_id
            (int16_t)(dx * 10),     // flow_x
            (int16_t)(dy * 10),     // flow_y
            0.0f,                   // flow_comp_m_x (ArduPilot computes this with onboard gyro)
            0.0f,                   // flow_comp_m_y
            quality,                // surface quality (0-255)
            -1.0f,                  // ground_distance (-1: unknown, ArduPilot scales with Baro/ToF)
            0.0f,                   // flow_rate_x
            0.0f,                   // flow_rate_y
            &s_mavlink_seq
        );

        // Transmit packet directly to StampFly via Grove Red UART (2Mbps)
        // Transmit packet simultaneously to StampFly (UART) and all connected GCS (Wi-Fi UDP)
        mavlink_router_send_internal(mav_packet, len, MAV_ROUTE_DEST_ALL);

        // Transmit periodic 1Hz HEARTBEAT (#0) so GCS and FC recognize CamS3 component
        static int64_t last_hb_us = 0;
        if (now_us - last_hb_us >= 1000000) {
            last_hb_us = now_us;
            uint8_t hb_packet[32];
            uint16_t hb_len = mavlink_pack_heartbeat_v2(hb_packet, &s_mavlink_seq);
            // Transmit periodic 1Hz HEARTBEAT to both StampFly and GCS
            mavlink_router_send_internal(hb_packet, hb_len, MAV_ROUTE_DEST_ALL);
        }

        // Free the previous frame and advance
        esp_camera_fb_return(fb_last);
        fb_last = fb_curr;

        frame_count++;

        // Periodic diagnostics logging (every 5 seconds)
        if (now_us - last_stats_us >= 5000000) {
            float fps = (float)frame_count * 1000000.0f / (float)(now_us - last_stats_us);
            s_latest_metrics.fps = fps;
            ESP_LOGI(TAG, "Optical Flow Active: %.1f FPS | dx=%d, dy=%d | Quality=%u/255 | UART Tx=%u bytes",
                     fps, dx, dy, quality, len);
            frame_count = 0;
            last_stats_us = now_us;
        }
    }

    if (fb_last) {
        esp_camera_fb_return(fb_last);
    }
    vTaskDelete(NULL);
}

// ==========================================
// Public API Functions
// ==========================================

esp_err_t optical_flow_init(void)
{
    ESP_LOGI(TAG, "Initializing camera for Optical Flow (Grayscale QQVGA 160x120)...");

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

        .pixel_format = PIXFORMAT_GRAYSCALE,   // Raw grayscale for instantaneous block matching
        .frame_size   = BM_FRAME_SIZE,         // 160x120
        .jpeg_quality = 12,
        .fb_count     = 3,                     // Triple buffering for continuous grab
        .fb_location  = CAMERA_FB_IN_DRAM,     // Internal RAM for fast CPU memory access
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Optical Flow camera init failed: 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        // Optimize for motion tracking
        s->set_brightness(s, 0);
        s->set_contrast(s, 1);
        s->set_saturation(s, -2);
    }

    ESP_LOGI(TAG, "Optical Flow Camera successfully initialized!");
    return ESP_OK;
}

esp_err_t optical_flow_start(void)
{
    if (s_running) {
        return ESP_OK;
    }
    s_running = true;

    // Pin optical flow processing task to Core 0 (leaving Core 1 for MAVLink telemetry bridge)
    BaseType_t res = xTaskCreatePinnedToCore(
        optical_flow_task,
        "optical_flow",
        4096,
        NULL,
        8,          // High priority for consistent frame timing
        NULL,
        0           // Core 0
    );

    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}

void optical_flow_get_latest(optical_flow_metrics_t *metrics)
{
    if (metrics) {
        *metrics = s_latest_metrics;
    }
}
