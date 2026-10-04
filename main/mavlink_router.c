#include "mavlink_router.h"
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "board_pins.h"

static const char *TAG = "mav_router";

static int s_udp_sock = -1;
static struct sockaddr_in s_client_addr;
static bool s_has_client_addr = false;
static SemaphoreHandle_t s_uart_tx_mutex = NULL;

static uint32_t s_bytes_uart_to_udp = 0;
static uint32_t s_bytes_udp_to_uart = 0;

static void uart_init_internal(void)
{
    // Detach pins from default USB/JTAG PHY and route to GPIO Matrix
    gpio_reset_pin(BOARD_PIN_UART_TX);
    gpio_reset_pin(BOARD_PIN_UART_RX);

    const uart_config_t uart_config = {
        .baud_rate = BOARD_UART_BAUDRATE, // 2,000,000 bps (2Mbps)
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 8KB RX buffer to cushion against Wi-Fi task jitter at 2Mbps
    ESP_ERROR_CHECK(uart_driver_install(BOARD_UART_PORT, 8192, 4096, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_UART_PORT, BOARD_PIN_UART_TX, BOARD_PIN_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d (Buffer=8KB)",
             BOARD_UART_PORT, BOARD_PIN_UART_TX, BOARD_PIN_UART_RX, BOARD_UART_BAUDRATE);
}

static void udp_init_internal(void)
{
    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_udp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        return;
    }

    int opt = 1;
    setsockopt(s_udp_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(s_udp_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));

    // 100ms receive timeout so recvfrom does not block indefinitely
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(s_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons(MAVLINK_ROUTER_UDP_PORT),
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
    s_client_addr.sin_port = htons(MAVLINK_ROUTER_UDP_PORT);
    inet_aton("192.168.4.255", &s_client_addr.sin_addr);

    ESP_LOGI(TAG, "UDP socket bound to port %d (broadcasting to 192.168.4.255:14550)", MAVLINK_ROUTER_UDP_PORT);
}

// Task: Reads from StampFly (UART) and sends via UDP to Phone / PC (Mission Planner)
static void uart_to_udp_task(void *pvParameters)
{
    uint8_t rx_buf[MAVLINK_ROUTER_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(BOARD_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(5));
        if (len > 0 && s_udp_sock >= 0) {
            // Non-blocking send: never freeze the task even if Wi-Fi buffers fill momentarily
            int err = sendto(s_udp_sock, rx_buf, len, MSG_DONTWAIT,
                             (struct sockaddr *)&s_client_addr, sizeof(s_client_addr));
            if (err > 0) {
                s_bytes_uart_to_udp += len;
            }
        }
    }
}

// Task: Reads from Phone / PC (UDP) and sends via UART to StampFly
static void udp_to_uart_task(void *pvParameters)
{
    uint8_t rx_buf[MAVLINK_ROUTER_BUF_SIZE];
    struct sockaddr_in from_addr;

    while (1) {
        if (s_udp_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        socklen_t socklen = sizeof(from_addr);
        int len = recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&from_addr, &socklen);
        if (len > 0) {
            // CRITICAL FIX: Learn the client IP, but ALWAYS keep destination port FIXED to 14550!
            // Mission Planner / GCS sends from an ephemeral outbound port, but its listener
            // is strictly bound to port 14550. Overwriting the port caused complete telemetry freeze!
            if (!s_has_client_addr ||
                s_client_addr.sin_addr.s_addr != from_addr.sin_addr.s_addr) {
                s_client_addr.sin_addr = from_addr.sin_addr;
                s_client_addr.sin_port = htons(MAVLINK_ROUTER_UDP_PORT); // Always 14550!
                s_has_client_addr = true;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntoa_r(from_addr.sin_addr, ip_str, sizeof(ip_str));
                ESP_LOGI(TAG, "Discovered active GCS at %s - sending telemetry to %s:%d",
                         ip_str, ip_str, MAVLINK_ROUTER_UDP_PORT);
            }

            // Write to StampFly UART with mutex protection
            if (xSemaphoreTake(s_uart_tx_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                uart_write_bytes(BOARD_UART_PORT, (const char *)rx_buf, len);
                xSemaphoreGive(s_uart_tx_mutex);
                s_bytes_udp_to_uart += len;
            }
        }
    }
}

// Task: Prints periodic throughput statistics
static void stats_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        char ip_str[INET_ADDRSTRLEN];
        inet_ntoa_r(s_client_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "MAVLink Stats (5s): FC->Phone: %lu bytes | Phone->FC: %lu bytes | Dest: %s:%d",
                 (unsigned long)s_bytes_uart_to_udp, (unsigned long)s_bytes_udp_to_uart,
                 ip_str, ntohs(s_client_addr.sin_port));
        s_bytes_uart_to_udp = 0;
        s_bytes_udp_to_uart = 0;
    }
}

// ----------------------------------------------------------------------
// Public APIs
// ----------------------------------------------------------------------

esp_err_t mavlink_router_init(void)
{
    s_uart_tx_mutex = xSemaphoreCreateMutex();
    if (!s_uart_tx_mutex) {
        ESP_LOGE(TAG, "Failed to create UART TX mutex");
        return ESP_ERR_NO_MEM;
    }

    uart_init_internal();
    udp_init_internal();

    // Pin MAVLink tasks to Core 1 (dedicated to real-time telemetry communication)
    xTaskCreatePinnedToCore(uart_to_udp_task, "uart_to_udp", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(udp_to_uart_task, "udp_to_uart", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(stats_task,       "bridge_stats", 2048, NULL, 1,  NULL, 1);

    ESP_LOGI(TAG, "MAVLink Router (Simple Bridge) started on Core 1");
    return ESP_OK;
}

esp_err_t mavlink_router_add_endpoint(const char *ip_str, uint16_t port, bool is_static)
{
    if (ip_str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    inet_aton(ip_str, &s_client_addr.sin_addr);
    s_client_addr.sin_port = htons(port);
    s_has_client_addr = true;
    ESP_LOGI(TAG, "Endpoint set to %s:%u", ip_str, port);
    return ESP_OK;
}

esp_err_t mavlink_router_send_internal(const uint8_t *packet, uint16_t length, mav_route_dest_t dest)
{
    if (packet == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Dispatch to StampFly Flight Controller via UART
    if (dest & MAV_ROUTE_DEST_FC) {
        if (xSemaphoreTake(s_uart_tx_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            uart_write_bytes(BOARD_UART_PORT, (const char *)packet, length);
            xSemaphoreGive(s_uart_tx_mutex);
        } else {
            return ESP_ERR_TIMEOUT;
        }
    }

    // 2. Dispatch to GCS endpoint via UDP
    if (dest & MAV_ROUTE_DEST_GCS) {
        if (s_udp_sock >= 0) {
            sendto(s_udp_sock, packet, length, MSG_DONTWAIT,
                   (struct sockaddr *)&s_client_addr, sizeof(s_client_addr));
        }
    }

    return ESP_OK;
}

int mavlink_router_get_active_client_count(void)
{
    return s_has_client_addr ? 1 : 0;
}
