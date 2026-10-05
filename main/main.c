#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "board_pins.h"
#include "wifi_ap.h"
#include "mavlink_router.h"
#include "optical_flow.h"
#include "camera_stream.h"

static const char *TAG = "main";

// Vision Mode Selection:
// 2: Step 1 Dummy Test (Fixed values for #100 OPTICAL_FLOW to verify comms with FC / GCS)
// 1: High-Speed Optical Flow Engine (QQVGA 160x120 Grayscale @ 30+ FPS, MAVLink #100)
// 0: HTTP MJPEG Video Streaming (VGA 640x480 JPEG @ http://192.168.4.1/stream)
#define VISION_MODE_STREAM        0
#define VISION_MODE_OPTICAL_FLOW  1
#define VISION_MODE_DUMMY_TEST    2

#define CONFIG_VISION_MODE        VISION_MODE_OPTICAL_FLOW

static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // In typical M5 circuits, LOW = ON, HIGH = OFF
    gpio_set_level(BOARD_PIN_LED, 1);
}

static void led_task(void *pvParameters)
{
    while (1) {
        if (wifi_ap_has_client()) {
            // Client connected: solid ON (LOW)
            gpio_set_level(BOARD_PIN_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // Waiting for client: slow blink
            gpio_set_level(BOARD_PIN_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(BOARD_PIN_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(900));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "  StampFly Companion Computer (Unit CamS3-5MP)");
    ESP_LOGI(TAG, "  - MAVLink Router (Core 1): Grove G19 (TX) / G20 (RX) @ 2Mbps <-> UDP 14550");
    ESP_LOGI(TAG, "  - Wi-Fi SoftAP: 192.168.4.1");
#if CONFIG_VISION_MODE == VISION_MODE_DUMMY_TEST
    ESP_LOGI(TAG, "  - Vision Mode (Core 0) : Step 1 DUMMY TEST (#100 OPTICAL_FLOW @ 10Hz, fixed dx=123, dy=-456)");
#elif CONFIG_VISION_MODE == VISION_MODE_OPTICAL_FLOW
    ESP_LOGI(TAG, "  - Vision Mode (Core 0) : Optical Flow Engine @ 30+ FPS (MAVLink #100)");
#else
    ESP_LOGI(TAG, "  - Vision Mode (Core 0) : HTTP Video Stream (http://192.168.4.1/stream)");
#endif
    ESP_LOGI(TAG, "===============================================");

    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Status LED
    led_init();
    xTaskCreate(led_task, "led_task", 2048, NULL, 1, NULL);

    // 3. Initialize Wi-Fi Access Point
    ESP_LOGI(TAG, "Starting Wi-Fi Access Point...");
    ESP_ERROR_CHECK(wifi_ap_init());
    ESP_LOGI(TAG, "  - Wi-Fi SoftAP active: SSID [%s] (192.168.4.1)", wifi_ap_get_ssid());

    // 4. Initialize MAVLink Multi-Endpoint Router on Core 1 (UART Grove G19/G20 <-> UDP 14550)
    ESP_LOGI(TAG, "Starting MAVLink Multi-Endpoint Router...");
    ESP_ERROR_CHECK(mavlink_router_init());

    // 5. Initialize Camera & Vision Engine on Core 0
#if CONFIG_VISION_MODE == VISION_MODE_DUMMY_TEST
    ESP_LOGI(TAG, "Starting Step 1 DUMMY TEST on Core 0 (Broadcasting #100 to FC & UDP)...");
    optical_flow_dummy_start();
#elif CONFIG_VISION_MODE == VISION_MODE_OPTICAL_FLOW
    ESP_LOGI(TAG, "Initializing Camera for High-Speed Optical Flow (160x120 Grayscale)...");
    esp_err_t err = optical_flow_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera hardware init returned error 0x%x. Engine will run in NG-Reporting Mode (quality=0).", err);
    } else {
        ESP_LOGI(TAG, "Camera hardware initialized successfully!");
    }
    // Unconditional start: NEVER fail to transmit telemetry!
    ESP_LOGI(TAG, "Starting Optical Flow Engine on Core 0 (Broadcasting #100 @ 20Hz to FC & UDP)...");
    optical_flow_start();
#elif CONFIG_VISION_MODE == VISION_MODE_STREAM
    ESP_LOGI(TAG, "Initializing 5MP Camera for Live Video Streaming...");
    if (camera_init() == ESP_OK) {
        ESP_LOGI(TAG, "Starting HTTP Video Streaming Server (/stream)...");
        camera_stream_start();
    } else {
        ESP_LOGE(TAG, "Camera initialization failed! Continuing in telemetry-only mode.");
    }
#endif

    ESP_LOGI(TAG, "Initialization complete. Ready for flight!");
}
