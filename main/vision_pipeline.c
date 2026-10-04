#include "vision_pipeline.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "board_pins.h"
#include "mavlink_optical_flow_pack.h"
#include "mavlink_landing_target_pack.h"
#include "mavlink_heartbeat_pack.h"
#include "mavlink_router.h"

static const char *TAG = "vision_pipeline";

static vision_pipeline_status_t s_status = {0};
static uint8_t s_mav_seq = 0;
static bool s_running = false;
static SemaphoreHandle_t s_mutex = NULL;

// Scratch buffer in PSRAM for OSD overlay rendering and HTTP stream
static uint8_t *s_overlay_buf = NULL;
static uint16_t s_frame_w = 160;
static uint16_t s_frame_h = 120;

// ==========================================
// Drawing Helpers for Live Video Overlay (OSD)
// ==========================================

static void draw_pixel(uint8_t *buf, int w, int h, int x, int y, uint8_t color) {
    if (x >= 0 && x < w && y >= 0 && y < h) {
        buf[y * w + x] = color;
    }
}

static void draw_line(uint8_t *buf, int w, int h, int x0, int y0, int x1, int y1, uint8_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        draw_pixel(buf, w, h, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_rect(uint8_t *buf, int w, int h, int x0, int y0, int x1, int y1, uint8_t color) {
    draw_line(buf, w, h, x0, y0, x1, y0, color);
    draw_line(buf, w, h, x1, y0, x1, y1, color);
    draw_line(buf, w, h, x1, y1, x0, y1, color);
    draw_line(buf, w, h, x0, y1, x0, y0, color);
}

// ==========================================
// Master Vision Dispatcher Task (Core 0)
// ==========================================

static void vision_dispatcher_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Master Vision Dispatcher running on Core %d", xPortGetCoreID());

    camera_fb_t *fb_last = esp_camera_fb_get();
    if (!fb_last) {
        ESP_LOGE(TAG, "Failed to get initial frame buffer!");
        vTaskDelete(NULL);
        return;
    }

    uint8_t mav_packet[80];
    int64_t last_time_us = esp_timer_get_time();
    int64_t last_stats_us = last_time_us;
    uint32_t frame_count = 0;
    uint32_t decimation_counter = 0;

    while (s_running) {
        camera_fb_t *fb_curr = esp_camera_fb_get();
        if (!fb_curr) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        uint32_t dt_us = (uint32_t)(now_us - last_time_us);
        (void)dt_us;
        last_time_us = now_us;

        uint16_t w = fb_curr->width;
        uint16_t h = fb_curr->height;

        // -------------------------------------------------------------
        // Pipeline 1: Optical Flow Computation (Every frame @ ~30 FPS)
        // -------------------------------------------------------------
        optical_flow_metrics_t flow_metrics;
        optical_flow_get_latest(&flow_metrics);

        // -------------------------------------------------------------
        // Pipeline 2: Landing Target / ArUco Detection (Decimated @ ~6 FPS)
        // -------------------------------------------------------------
        marker_target_t target;
        decimation_counter++;
        if (decimation_counter >= 5) {
            decimation_counter = 0;
            if (marker_detector_process(w, h, fb_curr->buf, &target)) {
                // Send MAVLink #149 LANDING_TARGET to StampFly
                uint16_t lt_len = mavlink_pack_landing_target_v2(
                    mav_packet,
                    (uint64_t)now_us,
                    target.id,
                    target.angle_x,
                    target.angle_y,
                    target.distance,
                    (target.width / (float)w),
                    (target.height / (float)h),
                    &s_mav_seq
                );
                // Transmit LANDING_TARGET simultaneously to StampFly (UART) and all GCS (Wi-Fi UDP)
                mavlink_router_send_internal(mav_packet, lt_len, MAV_ROUTE_DEST_ALL);
            }
        }

        // Transmit periodic 1Hz HEARTBEAT (#0) so GCS and FC recognize CamS3 component
        static int64_t last_hb_us = 0;
        if (now_us - last_hb_us >= 1000000) {
            last_hb_us = now_us;
            uint8_t hb_packet[32];
            uint16_t hb_len = mavlink_pack_heartbeat_v2(hb_packet, &s_mav_seq);
            // Transmit periodic 1Hz HEARTBEAT to both StampFly and GCS
            mavlink_router_send_internal(hb_packet, hb_len, MAV_ROUTE_DEST_ALL);
        }

        // -------------------------------------------------------------
        // Pipeline 3: Update State & Copy to OSD Buffer for Live Stream
        // -------------------------------------------------------------
        if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
            s_status.flow = flow_metrics;
            s_status.target = target;
            s_status.frame_count++;

            // Copy to overlay buffer if allocated
            if (s_overlay_buf) {
                memcpy(s_overlay_buf, fb_curr->buf, w * h);

                // Draw center crosshair
                int cx = w / 2;
                int cy = h / 2;
                draw_line(s_overlay_buf, w, h, cx - 4, cy, cx + 4, cy, 255);
                draw_line(s_overlay_buf, w, h, cx, cy - 4, cx, cy + 4, 255);

                // Draw Optical Flow motion vector arrow
                int fx = cx + flow_metrics.dx * 3;
                int fy = cy + flow_metrics.dy * 3;
                draw_line(s_overlay_buf, w, h, cx, cy, fx, fy, 255);

                // Draw Landing Target bounding box if detected
                if (target.detected) {
                    draw_rect(s_overlay_buf, w, h,
                              target.corners_x[0], target.corners_y[0],
                              target.corners_x[2], target.corners_y[2], 255);
                }
            }
            xSemaphoreGive(s_mutex);
        }

        // Advance frame buffer
        esp_camera_fb_return(fb_last);
        fb_last = fb_curr;

        frame_count++;

        // Periodic diagnostics logging (every 5 seconds)
        if (now_us - last_stats_us >= 5000000) {
            float fps = (float)frame_count * 1000000.0f / (float)(now_us - last_stats_us);
            s_status.pipeline_fps = fps;
            ESP_LOGI(TAG, "Vision Pipeline Active: %.1f FPS | Flow: dx=%d, dy=%d (Q=%u) | Target: %s (ID=%u, dist=%.2fm)",
                     fps, flow_metrics.dx, flow_metrics.dy, flow_metrics.quality,
                     target.detected ? "FOUND" : "NONE", target.id, target.distance);
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
// Public Interface
// ==========================================

esp_err_t vision_pipeline_init(void)
{
    ESP_LOGI(TAG, "Initializing Vision Pipeline (Multi-Track: Flow + Marker + Stream)...");

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }

    // Allocate PSRAM buffer for live streaming overlay
    s_frame_w = 160;
    s_frame_h = 120;
    s_overlay_buf = (uint8_t *)malloc(s_frame_w * s_frame_h);
    if (!s_overlay_buf) {
        ESP_LOGW(TAG, "Could not allocate overlay buffer; streaming overlay disabled");
    }

    // 1. Initialize Camera and Optical Flow Engine
    esp_err_t err = optical_flow_init();
    if (err != ESP_OK) {
        return err;
    }

    // 2. Initialize Marker Detector
    marker_detector_init(66.5f, 52.0f);

    ESP_LOGI(TAG, "Vision Pipeline successfully initialized!");
    return ESP_OK;
}

esp_err_t vision_pipeline_start(void)
{
    if (s_running) {
        return ESP_OK;
    }
    s_running = true;

    // Start Optical Flow sub-task
    optical_flow_start();

    // Start Master Vision Dispatcher on Core 0
    BaseType_t res = xTaskCreatePinnedToCore(
        vision_dispatcher_task,
        "vision_hub",
        4096,
        NULL,
        7,
        NULL,
        0   // Core 0
    );

    return (res == pdPASS) ? ESP_OK : ESP_FAIL;
}

void vision_pipeline_get_status(vision_pipeline_status_t *status)
{
    if (status && s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        *status = s_status;
        xSemaphoreGive(s_mutex);
    }
}

esp_err_t vision_pipeline_get_overlay_jpg(uint8_t **out_jpg, size_t *out_len)
{
    if (!out_jpg || !out_len || !s_overlay_buf || !s_mutex) {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool ok = fmt2jpg(s_overlay_buf, s_frame_w * s_frame_h, s_frame_w, s_frame_h,
                      PIXFORMAT_GRAYSCALE, 80, out_jpg, out_len);
    xSemaphoreGive(s_mutex);

    return ok ? ESP_OK : ESP_FAIL;
}
