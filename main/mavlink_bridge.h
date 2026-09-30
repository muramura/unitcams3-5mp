#pragma once

#include "esp_err.h"

#define MAVLINK_UDP_PORT        14550
#define MAVLINK_BUF_SIZE        1024

esp_err_t mavlink_bridge_init(void);
