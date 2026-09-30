#include "mavlink_bridge.h"
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "board_pins.h"

static const char *TAG = "mavlink_bridge";

static int s_udp_sock = -1;
static struct sockaddr_in s_client_addr;
static bool s_has_client_addr = false;

static uint32_t s_bytes_uart_to_udp = 0;
static uint32_t s_bytes_udp_to_uart = 0;

static void uart_init_internal(void)
{
    const uart_config_t uart_config = {
        .baud_rate = BOARD_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(BOARD_UART_PORT, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_UART_PORT, BOARD_PIN_UART_TX, BOARD_PIN_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d",
             BOARD_UART_PORT, BOARD_PIN_UART_TX, BOARD_PIN_UART_RX, BOARD_UART_BAUDRATE);
}

static void udp_init_internal(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        return;
    }

    // Enable Broadcast so initial packets can reach any connected phone
    int broadcast_en = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_en, sizeof(broadcast_en));

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(MAVLINK_UDP_PORT),
    };

    if (bind(s_udp_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(s_udp_sock);
        s_udp_sock = -1;
        return;
    }

    // Default destination: 192.168.4.255:14550 (SoftAP Broadcast)
    memset(&s_client_addr, 0, sizeof(s_client_addr));
    s_client_addr.sin_family = AF_INET;
    s_client_addr.sin_port = htons(MAVLINK_UDP_PORT);
    inet_aton("192.168.4.255", &s_client_addr.sin_addr);

    ESP_LOGI(TAG, "UDP socket bound to port %d (broadcasting to 192.168.4.255)", MAVLINK_UDP_PORT);
}

// Task: Reads from StampFly (UART) and sends via UDP to Phone (Mission Planner)
static void uart_to_udp_task(void *pvParameters)
{
    uint8_t rx_buf[MAVLINK_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(BOARD_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(10));
        if (len > 0 && s_udp_sock >= 0) {
            int err = sendto(s_udp_sock, rx_buf, len, 0,
                             (struct sockaddr *)&s_client_addr, sizeof(s_client_addr));
            if (err < 0) {
                ESP_LOGW(TAG, "UDP sendto failed: errno %d", errno);
            } else {
                s_bytes_uart_to_udp += len;
            }
        }
    }
}

// Task: Reads from Phone (UDP) and sends via UART to StampFly
static void udp_to_uart_task(void *pvParameters)
{
    uint8_t rx_buf[MAVLINK_BUF_SIZE];
    struct sockaddr_in from_addr;
    socklen_t socklen = sizeof(from_addr);

    while (1) {
        if (s_udp_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int len = recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&from_addr, &socklen);
        if (len > 0) {
            // Learn the exact client IP address and port from the first incoming packet
            if (!s_has_client_addr ||
                s_client_addr.sin_addr.s_addr != from_addr.sin_addr.s_addr ||
                s_client_addr.sin_port != from_addr.sin_port) {
                s_client_addr = from_addr;
                s_has_client_addr = true;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntoa_r(from_addr.sin_addr, ip_str, sizeof(ip_str));
                ESP_LOGI(TAG, "Discovered active GCS (Mission Planner) at %s:%d - switching to direct unicast",
                         ip_str, ntohs(from_addr.sin_port));
            }

            // Write to StampFly UART
            uart_write_bytes(BOARD_UART_PORT, (const char *)rx_buf, len);
            s_bytes_udp_to_uart += len;
        }
    }
}

// Task: Prints periodic throughput statistics
static void stats_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "MAVLink Bridge Stats (5s): FC->Phone: %lu bytes | Phone->FC: %lu bytes",
                 (unsigned long)s_bytes_uart_to_udp, (unsigned long)s_bytes_udp_to_uart);
        s_bytes_uart_to_udp = 0;
        s_bytes_udp_to_uart = 0;
    }
}

esp_err_t mavlink_bridge_init(void)
{
    uart_init_internal();
    udp_init_internal();

    // Pin MAVLink tasks to Core 1 (dedicated to real-time telemetry communication)
    xTaskCreatePinnedToCore(uart_to_udp_task, "uart_to_udp", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(udp_to_uart_task, "udp_to_uart", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(stats_task, "bridge_stats", 2048, NULL, 1, NULL, 1);

    ESP_LOGI(TAG, "MAVLink Bridge started on Core 1");
    return ESP_OK;
}
