#pragma once

#include "esp_err.h"

#define CAMERA_STREAM_PORT      80

esp_err_t camera_init(void);
esp_err_t camera_stream_start(void);
