#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define WIFI_AP_SSID            "StampFly"
#define WIFI_AP_PASS            "ardupilot123"
#define WIFI_AP_CHANNEL         6
#define WIFI_AP_MAX_CONN        4

esp_err_t wifi_ap_init(void);
const char *wifi_ap_get_ssid(void);
bool wifi_ap_has_client(void);
