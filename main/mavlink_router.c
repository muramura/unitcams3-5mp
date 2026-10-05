#include "mavlink_router.h"
#include <string.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "board_pins.h"
#include "mavlink_heartbeat_pack.h"

static const char *TAG = "mav_router";

#define MAVLINK_ROUTER_QUEUE_LEN    16
#define MAVLINK_MAX_PACKET_LEN      280

typedef struct {
    uint16_t len;
    uint8_t  data[MAVLINK_MAX_PACKET_LEN];
} mavlink_queue_item_t;

// Dual Tx FreeRTOS Queues (Decouples Producer/Consumer and prevents any task blocking)
static QueueHandle_t s_fc_tx_queue   = NULL; // Packets bound for StampFly FC (UART)
static QueueHandle_t s_wifi_tx_queue = NULL; // Packets bound for GCS / Phone (Wi-Fi UDP)

// Networking & Hardware state
static int s_udp_sock = -1;
static struct sockaddr_in s_client_addr;
static bool s_has_client_addr = false;

// Throughput and Queue Drop Statistics
static uint32_t s_bytes_to_fc = 0;
static uint32_t s_bytes_to_wifi = 0;
static uint32_t s_bytes_from_fc = 0;
static uint32_t s_bytes_from_wifi = 0;
static uint32_t s_fc_tx_drops = 0;
static uint32_t s_wifi_tx_drops = 0;

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

    // 8KB RX ring buffer managed by ESP-IDF driver hardware FIFO
    ESP_ERROR_CHECK(uart_driver_install(BOARD_UART_PORT, 8192, 4096, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_UART_PORT, BOARD_PIN_UART_TX, BOARD_PIN_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d (Rx Buffer=8KB)",
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

    // Default destination: 192.168.4.2:14550 (First connected client via SoftAP DHCP)
    // Avoid 192.168.4.255 broadcast: prevents UDP loopback to self and avoids Android broadcast filtering!
    memset(&s_client_addr, 0, sizeof(s_client_addr));
    s_client_addr.sin_family = AF_INET;
    s_client_addr.sin_port = htons(MAVLINK_ROUTER_UDP_PORT);
    inet_aton("192.168.4.2", &s_client_addr.sin_addr);
    s_has_client_addr = true;

    ESP_LOGI(TAG, "UDP socket bound to port %d (default unicast to 192.168.4.2:14550)", MAVLINK_ROUTER_UDP_PORT);
}

// ----------------------------------------------------------------------
// Task 1: UART RX Task (Reads telemetry from StampFly FC -> Wi-Fi Tx Queue)
// ----------------------------------------------------------------------
static void uart_rx_task(void *pvParameters)
{
    uint8_t rx_buf[MAVLINK_ROUTER_BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(BOARD_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(5));
        if (len > 0) {
            s_bytes_from_fc += len;

            // Forward telemetry chunks to GCS/Phone via Wi-Fi Tx Queue
            int offset = 0;
            while (offset < len) {
                int chunk = len - offset;
                if (chunk > MAVLINK_MAX_PACKET_LEN) {
                    chunk = MAVLINK_MAX_PACKET_LEN;
                }
                mavlink_queue_item_t item;
                item.len = chunk;
                memcpy(item.data, &rx_buf[offset], chunk);

                if (s_wifi_tx_queue) {
                    if (xQueueSend(s_wifi_tx_queue, &item, 0) != pdTRUE) {
                        s_wifi_tx_drops++;
                    }
                }
                offset += chunk;
            }
        }
    }
}

// ----------------------------------------------------------------------
// Task 2: UART TX Task (Sole writer to StampFly FC UART - No Mutex needed!)
// ----------------------------------------------------------------------
static void uart_tx_task(void *pvParameters)
{
    mavlink_queue_item_t item;

    while (1) {
        if (xQueueReceive(s_fc_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            int written = uart_write_bytes(BOARD_UART_PORT, (const char *)item.data, item.len);
            if (written > 0) {
                s_bytes_to_fc += written;
            }
        }
    }
}

// ----------------------------------------------------------------------
// Task 3: UDP RX Task (Reads commands/requests from GCS/Phone -> FC Tx Queue)
// ----------------------------------------------------------------------
static void udp_rx_task(void *pvParameters)
{
    uint8_t rx_buf[MAVLINK_ROUTER_BUF_SIZE];
    struct sockaddr_in from_addr;

    while (1) {
        if (s_udp_sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        socklen_t socklen = sizeof(from_addr);
        int len = recvfrom(s_udp_sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&from_addr, &socklen);
        if (len > 0) {
            // CRITICAL GUARD: Filter out self-loopback packets from CamS3 (192.168.4.1 or 127.0.0.1)
            char from_ip[INET_ADDRSTRLEN];
            inet_ntoa_r(from_addr.sin_addr, from_ip, sizeof(from_ip));
            if (strcmp(from_ip, "192.168.4.1") == 0 || strcmp(from_ip, "127.0.0.1") == 0) {
                continue; // Ignore self-echo!
            }

            s_bytes_from_wifi += len;

            // Learn client IP address, but keep destination port fixed to 14550!
            if (!s_has_client_addr ||
                s_client_addr.sin_addr.s_addr != from_addr.sin_addr.s_addr) {
                s_client_addr.sin_addr = from_addr.sin_addr;
                s_client_addr.sin_port = htons(MAVLINK_ROUTER_UDP_PORT);
                s_has_client_addr = true;
                ESP_LOGI(TAG, "Discovered active GCS at %s - sending telemetry to %s:%d",
                         from_ip, from_ip, MAVLINK_ROUTER_UDP_PORT);
            }

            // Forward commands to StampFly FC via FC Tx Queue
            int offset = 0;
            while (offset < len) {
                int chunk = len - offset;
                if (chunk > MAVLINK_MAX_PACKET_LEN) {
                    chunk = MAVLINK_MAX_PACKET_LEN;
                }
                mavlink_queue_item_t item;
                item.len = chunk;
                memcpy(item.data, &rx_buf[offset], chunk);

                if (s_fc_tx_queue) {
                    if (xQueueSend(s_fc_tx_queue, &item, 0) != pdTRUE) {
                        s_fc_tx_drops++;
                    }
                }
                offset += chunk;
            }
        }
    }
}

// ----------------------------------------------------------------------
// Task 4: Wi-Fi TX Task (Sole writer to UDP Socket - Absorbs all Wi-Fi jitter!)
// ----------------------------------------------------------------------
static void wifi_tx_task(void *pvParameters)
{
    mavlink_queue_item_t item;

    while (1) {
        if (xQueueReceive(s_wifi_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            if (s_udp_sock >= 0 && s_has_client_addr) {
                int sent = sendto(s_udp_sock, item.data, item.len, 0,
                                  (struct sockaddr *)&s_client_addr, sizeof(s_client_addr));
                if (sent > 0) {
                    s_bytes_to_wifi += sent;
                }
            }
        }
    }
}

// ----------------------------------------------------------------------
// Task 5: Stats Task (Periodic throughput and queue drop diagnostic logging)
// ----------------------------------------------------------------------
static void stats_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        char ip_str[INET_ADDRSTRLEN];
        inet_ntoa_r(s_client_addr.sin_addr, ip_str, sizeof(ip_str));
        ESP_LOGI(TAG, "Router (5s): FC->GCS: %lu B | GCS->FC: %lu B | Drops (FC: %lu, WiFi: %lu) | Dest: %s:%d",
                 (unsigned long)s_bytes_to_wifi, (unsigned long)s_bytes_to_fc,
                 (unsigned long)s_fc_tx_drops, (unsigned long)s_wifi_tx_drops,
                 ip_str, ntohs(s_client_addr.sin_port));
        s_bytes_to_wifi = 0;
        s_bytes_to_fc = 0;
    }
}

// ----------------------------------------------------------------------
// Task 6: Heartbeat Task (Unconditional 1Hz Companion Lifeline & Diagnostics)
// ----------------------------------------------------------------------
static void heartbeat_task(void *pvParameters)
{
    uint8_t seq = 0;
    uint8_t hb_packet[32];
    uint8_t stat_packet[64];

    // Announce boot to GCS
    vTaskDelay(pdMS_TO_TICKS(1500));
    uint16_t stat_len = mavlink_pack_statustext_v2(stat_packet, &seq, 6 /* INFO */, "CamS3: Router Active");
    mavlink_router_send_internal(stat_packet, stat_len, MAV_ROUTE_DEST_ALL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Strict 1Hz interval

        // Send Component 197 HEARTBEAT (#0) to both FC (UART) and GCS (UDP)
        uint16_t hb_len = mavlink_pack_heartbeat_v2(hb_packet, &seq);
        mavlink_router_send_internal(hb_packet, hb_len, MAV_ROUTE_DEST_ALL);
    }
}

// ----------------------------------------------------------------------
// Public APIs
// ----------------------------------------------------------------------

esp_err_t mavlink_router_init(void)
{
    // Create Dual Tx FreeRTOS Queues
    s_fc_tx_queue = xQueueCreate(MAVLINK_ROUTER_QUEUE_LEN, sizeof(mavlink_queue_item_t));
    s_wifi_tx_queue = xQueueCreate(MAVLINK_ROUTER_QUEUE_LEN, sizeof(mavlink_queue_item_t));

    if (!s_fc_tx_queue || !s_wifi_tx_queue) {
        ESP_LOGE(TAG, "Failed to allocate MAVLink router queues!");
        return ESP_ERR_NO_MEM;
    }

    uart_init_internal();
    udp_init_internal();

    // Pin all routing tasks to Core 1 (leaving Core 0 100% dedicated to Camera and Vision Pipelines)
    xTaskCreatePinnedToCore(uart_rx_task,   "uart_rx_task",   3072, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(uart_tx_task,   "uart_tx_task",   3072, NULL,  9, NULL, 1);
    xTaskCreatePinnedToCore(udp_rx_task,    "udp_rx_task",    3072, NULL,  9, NULL, 1);
    xTaskCreatePinnedToCore(wifi_tx_task,   "wifi_tx_task",   3072, NULL,  8, NULL, 1);
    xTaskCreatePinnedToCore(heartbeat_task, "heartbeat_task", 2560, NULL,  5, NULL, 1);
    xTaskCreatePinnedToCore(stats_task,     "router_stats",   2048, NULL,  1, NULL, 1);

    ESP_LOGI(TAG, "MAVLink Router (Dual-Queue + Independent 1Hz Heartbeat) started on Core 1");
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
    if (packet == NULL || length == 0 || length > MAVLINK_MAX_PACKET_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    mavlink_queue_item_t item;
    item.len = length;
    memcpy(item.data, packet, length);

    // 1. Dispatch to StampFly FC Queue (UART) - Non-blocking (0 timeout)
    if (dest & MAV_ROUTE_DEST_FC) {
        if (s_fc_tx_queue) {
            if (xQueueSend(s_fc_tx_queue, &item, 0) != pdTRUE) {
                s_fc_tx_drops++;
            }
        }
    }

    // 2. Dispatch to GCS Queue (Wi-Fi UDP) - Non-blocking (0 timeout)
    if (dest & MAV_ROUTE_DEST_GCS) {
        if (s_wifi_tx_queue) {
            if (xQueueSend(s_wifi_tx_queue, &item, 0) != pdTRUE) {
                s_wifi_tx_drops++;
            }
        }
    }

    return ESP_OK;
}

int mavlink_router_get_active_client_count(void)
{
    return s_has_client_addr ? 1 : 0;
}

void mavlink_router_get_stats(mavlink_router_stats_t *stats)
{
    if (stats) {
        stats->fc_tx_drops = s_fc_tx_drops;
        stats->wifi_tx_drops = s_wifi_tx_drops;
        stats->bytes_to_fc = s_bytes_to_fc;
        stats->bytes_to_wifi = s_bytes_to_wifi;
        stats->bytes_from_fc = s_bytes_from_fc;
        stats->bytes_from_wifi = s_bytes_from_wifi;
    }
}
