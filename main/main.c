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
    ESP_LOGI(TAG, "  - MAVLink Router (Core 1): Grove G19/G20 @ 2Mbps <-> UDP 14550");
    ESP_LOGI(TAG, "  - Wi-Fi SoftAP: 192.168.4.1");
    ESP_LOGI(TAG, "  - FPV Video Stream (Core 0): http://192.168.4.1/stream (On-Demand)");
    ESP_LOGI(TAG, "  - Optical Flow (Core 0): ARPS Engine (MAVLink #100 @ 20Hz)");
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

    // 5. Initialize Camera, Live Video Stream & Optical Flow Engine on Core 0
    ESP_LOGI(TAG, "Initializing PY260 Camera for Dual-Vision Pipeline (VGA JPEG)...");
    esp_err_t err = optical_flow_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera hardware init returned error 0x%x. Engine will run in NG-Reporting Mode (quality=0).", err);
    } else {
        ESP_LOGI(TAG, "Camera hardware initialized successfully!");
    }

    // Start HTTP Video Streaming Server on port 80 (On-Demand MJPEG)
    ESP_LOGI(TAG, "Starting HTTP Video Streaming Server (/stream)...");
    camera_stream_start();

    // Start Real-Time Optical Flow Engine on Core 0 (Broadcasting #100 @ 20Hz to FC & UDP)
    ESP_LOGI(TAG, "Starting Optical Flow Engine on Core 0...");
    optical_flow_start();

    ESP_LOGI(TAG, "Initialization complete. Ready for flight!");
}
