#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "board_pins.h"
#include "wifi_ap.h"
#include "mavlink_bridge.h"
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
    ESP_LOGI(TAG, "  - UART MAVLink Bridge: Grove G19 (TX) / G20 (RX)");
    ESP_LOGI(TAG, "  - Wi-Fi SoftAP: 192.168.4.1");
    ESP_LOGI(TAG, "  - Video Stream: http://192.168.4.1/stream");
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

    // 4. Initialize MAVLink Bridge (UART Grove G19/G20 <-> UDP 14550) on Core 1
    ESP_LOGI(TAG, "Starting MAVLink Bridge...");
    ESP_ERROR_CHECK(mavlink_bridge_init());

    // 5. Initialize Camera and Video Stream on Core 0
    ESP_LOGI(TAG, "Initializing 5MP Camera...");
    if (camera_init() == ESP_OK) {
        ESP_LOGI(TAG, "Starting HTTP Video Streaming Server...");
        camera_stream_start();
    } else {
        ESP_LOGE(TAG, "Camera initialization failed! Continuing in telemetry-only mode.");
    }

    ESP_LOGI(TAG, "Initialization complete. Ready for flight!");
}
