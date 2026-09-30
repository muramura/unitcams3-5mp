#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define WIFI_AP_SSID            "StampFly-Cam"
#define WIFI_AP_PASS            ""              // Open network by default (or set e.g. "stampfly")
#define WIFI_AP_CHANNEL         6
#define WIFI_AP_MAX_CONN        4

esp_err_t wifi_ap_init(void);
bool wifi_ap_has_client(void);
