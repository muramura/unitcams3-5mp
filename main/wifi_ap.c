#include "wifi_ap.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "wifi_ap";
static int s_active_clients = 0;
static char s_actual_ssid[33] = {0};

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        s_active_clients++;
        ESP_LOGI(TAG, "Client joined: " MACSTR " (aid=%d, active=%d)",
                 MAC2STR(event->mac), event->aid, s_active_clients);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        if (s_active_clients > 0) s_active_clients--;
        ESP_LOGI(TAG, "Client left: " MACSTR " (aid=%d, active=%d)",
                 MAC2STR(event->mac), event->aid, s_active_clients);
    }
}

esp_err_t wifi_ap_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    // Dynamically append MAC address lower 3 bytes (same mechanism as StampFly / TASK-015)
    uint8_t mac[6] = {0};
    esp_err_t ret_mac = esp_efuse_mac_get_custom(mac);
    if (ret_mac != ESP_OK) {
        ret_mac = esp_efuse_mac_get_default(mac);
    }
    snprintf(s_actual_ssid, sizeof(s_actual_ssid), "%s_%02X%02X%02X",
             WIFI_AP_SSID, mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = (strlen(WIFI_AP_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    strncpy((char *)wifi_config.ap.ssid, s_actual_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(s_actual_ssid);
    if (strlen(WIFI_AP_PASS) > 0) {
        strncpy((char *)wifi_config.ap.password, WIFI_AP_PASS, sizeof(wifi_config.ap.password) - 1);
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Print AP IP address (typically 192.168.4.1)
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(ap_netif, &ip_info);
    ESP_LOGI(TAG, "Wi-Fi SoftAP started. SSID: [%s] (Auth: %s) IP: " IPSTR,
             s_actual_ssid,
             (strlen(WIFI_AP_PASS) == 0) ? "OPEN" : "WPA2-PSK",
             IP2STR(&ip_info.ip));

    return ESP_OK;
}

const char *wifi_ap_get_ssid(void)
{
    return s_actual_ssid;
}

bool wifi_ap_has_client(void)
{
    return s_active_clients > 0;
}
