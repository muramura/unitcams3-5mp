#include "cams3_params.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "cams3_params";

#define NVS_NAMESPACE "cams3_p"

static cams3_param_entry_t s_params[] = {
    // Name (<=15 chars), value,  def,   min,    max,    type,                    readonly
    { "FLOW_SCALE",     1.0f,   1.0f,  1.0f,   3.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // 1: 160x120, 2: 320x240, 3: 640x480
    { "FLOW_CROP",      0.0f,   0.0f,  0.0f,   1.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // 0: Downsample, 1: Center Crop
    { "FLOW_PMAX",      7.0f,   7.0f,  4.0f,   16.0f,  CAMS3_PARAM_TYPE_REAL32, false }, // ARPS Search Radius
    { "FLOW_BSTEP",     4.0f,   4.0f,  2.0f,   8.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Block Step
    { "FLOW_QUAL_MIN",  20.0f,  20.0f, 0.0f,   255.0f, CAMS3_PARAM_TYPE_REAL32, false }, // Min Quality Threshold
    { "FLOW_RATE_HZ",   30.0f,  30.0f, 5.0f,   50.0f,  CAMS3_PARAM_TYPE_REAL32, false }, // Flow Update Rate
    { "FLOW_Z_EST_EN",  1.0f,   1.0f,  0.0f,   1.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Camera Altitude Estimation Enable
    { "FLOW_Z_DEF",     0.30f,  0.30f, 0.05f,  3.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Default Hover Height (m)
    { "CAM_JPEG_Q",     14.0f,  14.0f, 10.0f,  40.0f,  CAMS3_PARAM_TYPE_REAL32, false }, // JPEG Quality (10=High, 40=Low)
    { "CAM_VFLIP",      1.0f,   1.0f,  0.0f,   1.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Camera Vertical Flip
    { "CAM_HMIRROR",    0.0f,   0.0f,  0.0f,   1.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Camera Horizontal Mirror
    { "LAND_TGT_EN",    1.0f,   1.0f,  0.0f,   1.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Marker Landing Detector Enable
    { "LAND_RATE_HZ",   6.0f,   6.0f,  1.0f,   20.0f,  CAMS3_PARAM_TYPE_REAL32, false }, // Marker Detection Rate
    { "OSD_ENABLE",     1.0f,   1.0f,  0.0f,   1.0f,   CAMS3_PARAM_TYPE_REAL32, false }, // Video Stream Overlay
    { "MARK_SIZE_MM",  50.0f,  50.0f, 10.0f,  500.0f, CAMS3_PARAM_TYPE_REAL32, false }, // Marker Physical Size (mm)
};

#define PARAM_COUNT (sizeof(s_params) / sizeof(s_params[0]))

static SemaphoreHandle_t s_param_mutex = NULL;
static cams3_param_change_cb_t s_change_cb = NULL;

static void apply_hardware_setting(const char *name, float value)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;

    if (strcmp(name, "CAM_VFLIP") == 0) {
        s->set_vflip(s, (int)value);
        ESP_LOGI(TAG, "Applied Camera VFLIP: %d", (int)value);
    } else if (strcmp(name, "CAM_HMIRROR") == 0) {
        s->set_hmirror(s, (int)value);
        ESP_LOGI(TAG, "Applied Camera HMIRROR: %d", (int)value);
    } else if (strcmp(name, "CAM_JPEG_Q") == 0) {
        s->set_quality(s, (int)value);
        ESP_LOGI(TAG, "Applied Camera JPEG Quality: %d", (int)value);
    }
}

static esp_err_t save_param_to_nvs(const char *name, float val)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: 0x%x", err);
        return err;
    }

    uint32_t raw_bits;
    memcpy(&raw_bits, &val, sizeof(float));
    err = nvs_set_u32(handle, name, raw_bits);
    if (err == ESP_OK) {
        nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t cams3_params_init(void)
{
    if (s_param_mutex == NULL) {
        s_param_mutex = xSemaphoreCreateMutex();
    }

    // Initialize NVS flash partition
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing and re-initializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS Flash init failed: 0x%x", ret);
        return ret;
    }

    // Load or populate default parameters
    nvs_handle_t handle;
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace '%s': 0x%x", NVS_NAMESPACE, ret);
        return ret;
    }

    for (size_t i = 0; i < PARAM_COUNT; i++) {
        uint32_t raw_bits = 0;
        esp_err_t err = nvs_get_u32(handle, s_params[i].name, &raw_bits);
        if (err == ESP_OK) {
            float loaded_val;
            memcpy(&loaded_val, &raw_bits, sizeof(float));
            // Validate bounds
            if (loaded_val >= s_params[i].min_val && loaded_val <= s_params[i].max_val) {
                s_params[i].value = loaded_val;
                ESP_LOGI(TAG, "Loaded param '%s' = %.3f from NVS", s_params[i].name, loaded_val);
            } else {
                ESP_LOGW(TAG, "Param '%s' out of bounds (%.3f), resetting to default %.3f",
                         s_params[i].name, loaded_val, s_params[i].default_val);
                s_params[i].value = s_params[i].default_val;
                save_param_to_nvs(s_params[i].name, s_params[i].default_val);
            }
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            // Write default value
            s_params[i].value = s_params[i].default_val;
            save_param_to_nvs(s_params[i].name, s_params[i].default_val);
            ESP_LOGI(TAG, "Initialized default param '%s' = %.3f in NVS", s_params[i].name, s_params[i].value);
        }

        // Apply hardware settings at boot if applicable
        apply_hardware_setting(s_params[i].name, s_params[i].value);
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "CamS3 parameter manager initialized (%u parameters ready for Web Console)", (unsigned)PARAM_COUNT);
    return ESP_OK;
}

float cams3_param_get(const char *name)
{
    if (!name) return 0.0f;

    float val = 0.0f;
    if (s_param_mutex && xSemaphoreTake(s_param_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (size_t i = 0; i < PARAM_COUNT; i++) {
            if (strncmp(s_params[i].name, name, 16) == 0) {
                val = s_params[i].value;
                break;
            }
        }
        xSemaphoreGive(s_param_mutex);
    }
    return val;
}

int32_t cams3_param_get_int(const char *name)
{
    return (int32_t)cams3_param_get(name);
}

esp_err_t cams3_param_set(const char *name, float value)
{
    if (!name) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ESP_ERR_NOT_FOUND;
    bool changed = false;

    if (s_param_mutex && xSemaphoreTake(s_param_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        for (size_t i = 0; i < PARAM_COUNT; i++) {
            if (strncmp(s_params[i].name, name, 16) == 0) {
                if (s_params[i].readonly) {
                    ret = ESP_ERR_INVALID_ARG;
                    break;
                }
                // Range clamping
                if (value < s_params[i].min_val) value = s_params[i].min_val;
                if (value > s_params[i].max_val) value = s_params[i].max_val;

                s_params[i].value = value;
                save_param_to_nvs(name, value);
                apply_hardware_setting(name, value);
                changed = true;
                ret = ESP_OK;
                break;
            }
        }
        xSemaphoreGive(s_param_mutex);
    }

    if (changed && s_change_cb) {
        s_change_cb(name, value);
    }

    return ret;
}

esp_err_t cams3_params_reset_defaults(void)
{
    if (s_param_mutex && xSemaphoreTake(s_param_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        for (size_t i = 0; i < PARAM_COUNT; i++) {
            s_params[i].value = s_params[i].default_val;
            save_param_to_nvs(s_params[i].name, s_params[i].default_val);
            apply_hardware_setting(s_params[i].name, s_params[i].default_val);
            if (s_change_cb) {
                s_change_cb(s_params[i].name, s_params[i].default_val);
            }
        }
        xSemaphoreGive(s_param_mutex);
    }
    ESP_LOGI(TAG, "All parameters reset to factory defaults");
    return ESP_OK;
}

esp_err_t cams3_params_save_all(void)
{
    if (s_param_mutex && xSemaphoreTake(s_param_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        for (size_t i = 0; i < PARAM_COUNT; i++) {
            save_param_to_nvs(s_params[i].name, s_params[i].value);
        }
        xSemaphoreGive(s_param_mutex);
    }
    return ESP_OK;
}

void cams3_params_register_change_callback(cams3_param_change_cb_t cb)
{
    s_change_cb = cb;
}

uint16_t cams3_params_get_count(void)
{
    return (uint16_t)PARAM_COUNT;
}

bool cams3_params_get_by_index(uint16_t index, cams3_param_entry_t *out_entry)
{
    if (index >= PARAM_COUNT || !out_entry) return false;

    if (s_param_mutex && xSemaphoreTake(s_param_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(out_entry, &s_params[index], sizeof(cams3_param_entry_t));
        xSemaphoreGive(s_param_mutex);
        return true;
    }
    return false;
}

esp_err_t cams3_params_to_json(char *buf, size_t max_len)
{
    if (!buf || max_len < 32) return ESP_ERR_INVALID_ARG;

    if (s_param_mutex && xSemaphoreTake(s_param_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    size_t offset = 0;
    int written = snprintf(buf + offset, max_len - offset, "[");
    if (written < 0 || written >= (max_len - offset)) {
        xSemaphoreGive(s_param_mutex);
        return ESP_ERR_NO_MEM;
    }
    offset += written;

    for (size_t i = 0; i < PARAM_COUNT; i++) {
        written = snprintf(buf + offset, max_len - offset,
                           "%s{\"name\":\"%s\",\"val\":%.4f,\"def\":%.4f,\"min\":%.4f,\"max\":%.4f,\"ro\":%d}",
                           (i > 0) ? "," : "",
                           s_params[i].name,
                           s_params[i].value,
                           s_params[i].default_val,
                           s_params[i].min_val,
                           s_params[i].max_val,
                           s_params[i].readonly ? 1 : 0);
        if (written < 0 || written >= (max_len - offset)) {
            xSemaphoreGive(s_param_mutex);
            return ESP_ERR_NO_MEM;
        }
        offset += written;
    }

    written = snprintf(buf + offset, max_len - offset, "]");
    if (written < 0 || written >= (max_len - offset)) {
        xSemaphoreGive(s_param_mutex);
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(s_param_mutex);
    return ESP_OK;
}
