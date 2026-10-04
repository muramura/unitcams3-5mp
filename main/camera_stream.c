#include "camera_stream.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board_pins.h"

static const char *TAG = "camera_stream";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static httpd_handle_t s_stream_httpd = NULL;

esp_err_t camera_init(void)
{
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

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_VGA,       // 640x480
        .jpeg_quality = 14,                  // Optimized: ~22KB/frame for silky smooth 25-30fps over Wi-Fi
        .fb_count     = 2,                   // Double buffer in PSRAM
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 0);
        ESP_LOGI(TAG, "Camera sensor initialized successfully (PID: 0x%04x)", s->id.PID);
    }

    return ESP_OK;
}

// Handler: MJPEG Video Streaming (/stream)
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[128];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Optimize TCP socket for real-time video streaming
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0) {
        int enable = 1;
        setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    }

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        // Combine boundary and part header into a single chunk send
        size_t hlen = snprintf(part_buf, sizeof(part_buf),
                               "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                               fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }

        esp_camera_fb_return(fb);
        fb = NULL;

        if (res != ESP_OK) {
            break;
        }

        // Yield briefly so Wi-Fi stack and lwIP buffers process cleanly without stutter
        vTaskDelay(pdMS_TO_TICKS(8));
    }

    return res;
}

// Handler: Single JPEG Snapshot (/capture)
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// Handler: Simple Web HUD Index Page (/)
static const char INDEX_HTML[] =
    "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>StampFly FPV & CC</title>"
    "<style>"
    "body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center;}"
    "h2{margin:12px 0 6px 0;font-size:18px;color:#00d2ff;}"
    ".status{font-size:13px;color:#aaa;margin-bottom:10px;}"
    ".video-box{max-width:95vw;max-height:75vh;border-radius:8px;box-shadow:0 4px 16px rgba(0,210,255,0.2);}"
    ".info{margin-top:12px;font-size:12px;color:#777;}"
    "</style></head><body>"
    "<h2>🚁 StampFly Companion Camera</h2>"
    "<div class=\"status\">Wi-Fi: Connected | MAVLink: UDP 14550 Broadcast | Stream: /stream</div>"
    "<img class=\"video-box\" src=\"/stream\" alt=\"Live Video Stream\"/>"
    "<div class=\"info\">Use <code>http://192.168.4.1/stream</code> in Mission Planner Video Settings</div>"
    "</body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

esp_err_t camera_stream_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CAMERA_STREAM_PORT;
    config.ctrl_port = CAMERA_STREAM_PORT + 1000;
    config.stack_size = 8192;
    config.core_id = 0; // Pin HTTP/Camera Stream task to Core 0

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG, "Starting HTTP Stream Server on port %d...", config.server_port);
    if (httpd_start(&s_stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(s_stream_httpd, &index_uri);
        httpd_register_uri_handler(s_stream_httpd, &stream_uri);
        httpd_register_uri_handler(s_stream_httpd, &capture_uri);
        ESP_LOGI(TAG, "Stream Server ready. Live view at http://192.168.4.1/stream");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server!");
    return ESP_FAIL;
}
