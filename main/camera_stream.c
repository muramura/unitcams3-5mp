#include "camera_stream.h"
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "cams3_params.h"
#include "web_page.h"

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

// Handler: Web Console & HUD Index Page (/)
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// Handler: GET /api/params
static esp_err_t api_params_get_handler(httpd_req_t *req)
{
    char *json_buf = (char *)malloc(2048);
    if (!json_buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    esp_err_t err = cams3_params_to_json(json_buf, 2048);
    if (err != ESP_OK) {
        free(json_buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t res = httpd_resp_send(req, json_buf, strlen(json_buf));
    free(json_buf);
    return res;
}

// Handler: POST & GET /api/param
static esp_err_t api_param_set_handler(httpd_req_t *req)
{
    char param_name[32] = {0};
    char param_val_str[32] = {0};
    char buf[128] = {0};

    // 1. Check URL query string (?name=...&val=...)
    size_t query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0 && query_len < sizeof(buf)) {
        if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
            httpd_query_key_value(buf, "name", param_name, sizeof(param_name));
            httpd_query_key_value(buf, "val", param_val_str, sizeof(param_val_str));
        }
    }

    // 2. If not found in query, read request body
    if (strlen(param_name) == 0 && req->content_len > 0 && req->content_len < sizeof(buf)) {
        int ret = httpd_req_recv(req, buf, req->content_len);
        if (ret > 0) {
            buf[ret] = '\0';
            if (httpd_query_key_value(buf, "name", param_name, sizeof(param_name)) != ESP_OK) {
                // Try simple JSON parse
                char *name_ptr = strstr(buf, "\"name\":");
                if (name_ptr) {
                    sscanf(name_ptr, "\"name\":\"%31[^\"]\"", param_name);
                }
                char *val_ptr = strstr(buf, "\"val\":");
                if (val_ptr) {
                    sscanf(val_ptr, "\"val\":%31[^,}]", param_val_str);
                }
            } else {
                httpd_query_key_value(buf, "val", param_val_str, sizeof(param_val_str));
            }
        }
    }

    if (strlen(param_name) == 0 || strlen(param_val_str) == 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, "{\"status\":\"error\",\"msg\":\"missing name or val\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    float val = (float)atof(param_val_str);
    esp_err_t set_err = cams3_param_set(param_name, val);
    if (set_err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        char err_res[96];
        snprintf(err_res, sizeof(err_res), "{\"status\":\"error\",\"code\":%d}", set_err);
        httpd_resp_send(req, err_res, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Web API updated param '%s' = %.4f", param_name, val);

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"name\":\"%s\",\"val\":%.4f}", param_name, val);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// Handler: POST /api/reset
static esp_err_t api_reset_handler(httpd_req_t *req)
{
    cams3_params_reset_defaults();
    ESP_LOGI(TAG, "Web API reset all parameters to defaults");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\",\"msg\":\"defaults restored\"}", HTTPD_RESP_USE_STRLEN);
}

// Handler: POST /api/reboot
static void reboot_timer_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t api_reboot_handler(httpd_req_t *req)
{
    ESP_LOGW(TAG, "Web API requested device reboot");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"status\":\"ok\",\"msg\":\"rebooting in 1s\"}", HTTPD_RESP_USE_STRLEN);

    // Spawn detached task to reboot after 1 second
    xTaskCreate(reboot_timer_task, "reboot_timer", 2048, NULL, 5, NULL);
    return ESP_OK;
}

// Handler: OPTIONS /api/* (CORS Preflight)
static esp_err_t api_options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
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
    config.uri_match_fn = httpd_uri_match_wildcard;

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

    httpd_uri_t api_params_get_uri = {
        .uri       = "/api/params",
        .method    = HTTP_GET,
        .handler   = api_params_get_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t api_param_set_uri = {
        .uri       = "/api/param",
        .method    = HTTP_POST,
        .handler   = api_param_set_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t api_param_get_set_uri = {
        .uri       = "/api/param",
        .method    = HTTP_GET,
        .handler   = api_param_set_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t api_reset_uri = {
        .uri       = "/api/reset",
        .method    = HTTP_POST,
        .handler   = api_reset_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t api_reboot_uri = {
        .uri       = "/api/reboot",
        .method    = HTTP_POST,
        .handler   = api_reboot_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t api_options_uri = {
        .uri       = "/api/*",
        .method    = HTTP_OPTIONS,
        .handler   = api_options_handler,
        .user_ctx  = NULL
    };

    ESP_LOGI(TAG, "Starting HTTP Console & Stream Server on port %d...", config.server_port);
    if (httpd_start(&s_stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(s_stream_httpd, &index_uri);
        httpd_register_uri_handler(s_stream_httpd, &stream_uri);
        httpd_register_uri_handler(s_stream_httpd, &capture_uri);
        httpd_register_uri_handler(s_stream_httpd, &api_params_get_uri);
        httpd_register_uri_handler(s_stream_httpd, &api_param_set_uri);
        httpd_register_uri_handler(s_stream_httpd, &api_param_get_set_uri);
        httpd_register_uri_handler(s_stream_httpd, &api_reset_uri);
        httpd_register_uri_handler(s_stream_httpd, &api_reboot_uri);
        httpd_register_uri_handler(s_stream_httpd, &api_options_uri);
        ESP_LOGI(TAG, "Web Console ready: http://192.168.4.1/ (API: /api/params, /api/param)");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server!");
    return ESP_FAIL;
}
