#include "camera_stream.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "camera_stream";

#define MAX_STREAM_FRAME_SIZE   (64 * 1024)  // 64 KB (VGA JPEG is typically 15-25 KB)
#define PART_BOUNDARY           "123456789000000000000987654321"

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static httpd_handle_t s_stream_httpd = NULL;

// Lock-free Ping-Pong frame buffers in PSRAM
static uint8_t *s_ping_pong_buf[2] = {NULL, NULL};
static size_t s_ping_pong_len[2] = {0, 0};
static volatile int s_latest_idx = -1;

// Active client counter for on-demand resource optimization
static volatile int s_active_clients = 0;
static SemaphoreHandle_t s_frame_ready_sem = NULL;

bool camera_stream_has_client(void)
{
    return (s_active_clients > 0);
}

void camera_stream_push_frame(const uint8_t *buf, size_t len)
{
    if (!s_ping_pong_buf[0] || !buf || len == 0 || len > MAX_STREAM_FRAME_SIZE) {
        return;
    }

    // Ping-pong: write to the inactive buffer
    int write_idx = (s_latest_idx == 0) ? 1 : 0;
    memcpy(s_ping_pong_buf[write_idx], buf, len);
    s_ping_pong_len[write_idx] = len;

    // Atomically publish latest buffer index
    s_latest_idx = write_idx;

    // Notify streaming handler
    if (s_frame_ready_sem != NULL) {
        xSemaphoreGive(s_frame_ready_sem);
    }
}

// Handler: MJPEG Video Streaming (/stream)
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    char part_buf[128];

    s_active_clients++;
    ESP_LOGI(TAG, "New video stream client connected (active: %d)", s_active_clients);

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        s_active_clients--;
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Optimize TCP socket for lowest latency real-time streaming
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0) {
        int enable = 1;
        setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    }

    while (true) {
        // Wait for next frame ready from optical flow loop (up to 250ms)
        if (xSemaphoreTake(s_frame_ready_sem, pdMS_TO_TICKS(250)) == pdTRUE) {
            int read_idx = s_latest_idx;
            if (read_idx >= 0 && s_ping_pong_len[read_idx] > 0) {
                size_t flen = s_ping_pong_len[read_idx];

                size_t hlen = snprintf(part_buf, sizeof(part_buf),
                                       "\r\n--" PART_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                       (unsigned int)flen);
                res = httpd_resp_send_chunk(req, part_buf, hlen);
                if (res == ESP_OK) {
                    res = httpd_resp_send_chunk(req, (const char *)s_ping_pong_buf[read_idx], flen);
                }

                if (res != ESP_OK) {
                    break;
                }
            }
        } else {
            // Heartbeat check: send empty boundary or check socket health if idle
            if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
                break;
            }
        }
    }

    s_active_clients--;
    if (s_active_clients < 0) s_active_clients = 0;
    ESP_LOGI(TAG, "Video stream client disconnected (active: %d)", s_active_clients);

    return res;
}

// Handler: Single JPEG Snapshot (/capture)
static esp_err_t capture_handler(httpd_req_t *req)
{
    int idx = s_latest_idx;
    if (idx < 0 || s_ping_pong_len[idx] == 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    return httpd_resp_send(req, (const char *)s_ping_pong_buf[idx], s_ping_pong_len[idx]);
}

// Handler: Web HUD Index Page (/)
static const char INDEX_HTML[] =
    "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>StampFly FPV & Optical Flow</title>"
    "<style>"
    "body{margin:0;background:#111;color:#eee;font-family:sans-serif;text-align:center;}"
    "h2{margin:12px 0 6px 0;font-size:18px;color:#00d2ff;}"
    ".status{font-size:13px;color:#aaa;margin-bottom:10px;}"
    ".video-box{max-width:95vw;max-height:75vh;border-radius:8px;box-shadow:0 4px 16px rgba(0,210,255,0.2);}"
    ".info{margin-top:12px;font-size:12px;color:#777;}"
    "</style></head><body>"
    "<h2>🚁 StampFly FPV & Optical Flow HUD</h2>"
    "<div class=\"status\">Wi-Fi: Connected | MAVLink #100: Active | Stream: /stream</div>"
    "<img class=\"video-box\" src=\"/stream\" alt=\"Live Video Stream\"/>"
    "<div class=\"info\">In Mission Planner: Right-click HUD -&gt; <b>Set MJPEG Source</b> -&gt; <code>http://192.168.4.1/stream</code></div>"
    "</body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

esp_err_t camera_stream_start(void)
{
    // Allocate double buffers in PSRAM
    if (s_ping_pong_buf[0] == NULL) {
        s_ping_pong_buf[0] = (uint8_t *)heap_caps_malloc(MAX_STREAM_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_ping_pong_buf[0]) s_ping_pong_buf[0] = (uint8_t *)malloc(MAX_STREAM_FRAME_SIZE);
    }
    if (s_ping_pong_buf[1] == NULL) {
        s_ping_pong_buf[1] = (uint8_t *)heap_caps_malloc(MAX_STREAM_FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_ping_pong_buf[1]) s_ping_pong_buf[1] = (uint8_t *)malloc(MAX_STREAM_FRAME_SIZE);
    }

    if (!s_ping_pong_buf[0] || !s_ping_pong_buf[1]) {
        ESP_LOGE(TAG, "Failed to allocate memory for streaming ping-pong buffers!");
        return ESP_ERR_NO_MEM;
    }

    if (s_frame_ready_sem == NULL) {
        s_frame_ready_sem = xSemaphoreCreateBinary();
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CAMERA_STREAM_PORT;
    config.ctrl_port = CAMERA_STREAM_PORT + 1000;
    config.stack_size = 8192;
    config.core_id = 0; // Run HTTP streaming on Core 0 alongside vision pipeline

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
