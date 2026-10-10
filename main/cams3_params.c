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
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#include "mavlink_router.h"

static const char *TAG = "cams3_params";

#define NVS_NAMESPACE "cams3_p"

static cams3_param_entry_t s_params[] = {
    // Name (<=15 chars), value,  def,   min,    max,    type,                  readonly
    { "FLOW_SCALE",     1.0f,   1.0f,  1.0f,   3.0f,   MAV_PARAM_TYPE_REAL32, false }, // 1: 160x120, 2: 320x240, 3: 640x480
    { "FLOW_CROP",      0.0f,   0.0f,  0.0f,   1.0f,   MAV_PARAM_TYPE_REAL32, false }, // 0: Downsample, 1: Center Crop
    { "FLOW_PMAX",      7.0f,   7.0f,  4.0f,   16.0f,  MAV_PARAM_TYPE_REAL32, false }, // ARPS Search Radius
    { "FLOW_BSTEP",     4.0f,   4.0f,  2.0f,   8.0f,   MAV_PARAM_TYPE_REAL32, false }, // Block Step
    { "FLOW_QUAL_MIN",  20.0f,  20.0f, 0.0f,   255.0f, MAV_PARAM_TYPE_REAL32, false }, // Min Quality Threshold
    { "FLOW_RATE_HZ",   30.0f,  30.0f, 5.0f,   50.0f,  MAV_PARAM_TYPE_REAL32, false }, // Flow Update Rate
    { "FLOW_Z_EST_EN",  1.0f,   1.0f,  0.0f,   1.0f,   MAV_PARAM_TYPE_REAL32, false }, // Camera Altitude Estimation Enable
    { "FLOW_Z_DEF",     0.30f,  0.30f, 0.05f,  3.0f,   MAV_PARAM_TYPE_REAL32, false }, // Default Hover Height (m)
    { "CAM_JPEG_Q",     14.0f,  14.0f, 10.0f,  40.0f,  MAV_PARAM_TYPE_REAL32, false }, // JPEG Quality (10=High, 40=Low)
    { "CAM_VFLIP",      1.0f,   1.0f,  0.0f,   1.0f,   MAV_PARAM_TYPE_REAL32, false }, // Camera Vertical Flip
    { "CAM_HMIRROR",    0.0f,   0.0f,  0.0f,   1.0f,   MAV_PARAM_TYPE_REAL32, false }, // Camera Horizontal Mirror
    { "LAND_TGT_EN",    1.0f,   1.0f,  0.0f,   1.0f,   MAV_PARAM_TYPE_REAL32, false }, // Marker Landing Detector Enable
    { "LAND_RATE_HZ",   6.0f,   6.0f,  1.0f,   20.0f,  MAV_PARAM_TYPE_REAL32, false }, // Marker Detection Rate
    { "OSD_ENABLE",     1.0f,   1.0f,  0.0f,   1.0f,   MAV_PARAM_TYPE_REAL32, false }, // Video Stream Overlay
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
        ESP_LOGE(TAG, "Could not open NVS namespace %s: 0x%x", NVS_NAMESPACE, ret);
        return ret;
    }

    xSemaphoreTake(s_param_mutex, portMAX_DELAY);
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        uint32_t raw_bits = 0;
        esp_err_t read_err = nvs_get_u32(handle, s_params[i].name, &raw_bits);
        if (read_err == ESP_OK) {
            float val;
            memcpy(&val, &raw_bits, sizeof(float));
            // Clamp within valid boundaries
            if (val < s_params[i].min_val) val = s_params[i].min_val;
            if (val > s_params[i].max_val) val = s_params[i].max_val;
            s_params[i].value = val;
            ESP_LOGI(TAG, "Loaded param: %s = %.2f (NVS)", s_params[i].name, s_params[i].value);
        } else {
            // Key not present yet: initialize with default
            s_params[i].value = s_params[i].default_val;
            memcpy(&raw_bits, &s_params[i].default_val, sizeof(float));
            nvs_set_u32(handle, s_params[i].name, raw_bits);
            ESP_LOGI(TAG, "Initialized default param: %s = %.2f", s_params[i].name, s_params[i].value);
        }
        apply_hardware_setting(s_params[i].name, s_params[i].value);
    }
    nvs_commit(handle);
    nvs_close(handle);
    xSemaphoreGive(s_param_mutex);

    ESP_LOGI(TAG, "CamS3 Parameter Subsystem initialized with %u parameters", (unsigned)PARAM_COUNT);
    return ESP_OK;
}

float cams3_param_get(const char *name)
{
    if (!name) return 0.0f;
    float res = 0.0f;

    if (s_param_mutex) xSemaphoreTake(s_param_mutex, portMAX_DELAY);
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(s_params[i].name, name, 16) == 0) {
            res = s_params[i].value;
            break;
        }
    }
    if (s_param_mutex) xSemaphoreGive(s_param_mutex);
    return res;
}

int32_t cams3_param_get_int(const char *name)
{
    return (int32_t)cams3_param_get(name);
}

esp_err_t cams3_param_set(const char *name, float value)
{
    if (!name) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ESP_ERR_NOT_FOUND;

    if (s_param_mutex) xSemaphoreTake(s_param_mutex, portMAX_DELAY);
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(s_params[i].name, name, 16) == 0) {
            if (s_params[i].readonly) {
                ret = ESP_ERR_INVALID_ARG;
                break;
            }
            if (value < s_params[i].min_val) value = s_params[i].min_val;
            if (value > s_params[i].max_val) value = s_params[i].max_val;

            s_params[i].value = value;
            save_param_to_nvs(s_params[i].name, value);
            apply_hardware_setting(s_params[i].name, value);

            if (s_change_cb) {
                s_change_cb(s_params[i].name, value);
            }
            ret = ESP_OK;
            ESP_LOGI(TAG, "Set & Saved param: %s = %.2f", s_params[i].name, value);
            break;
        }
    }
    if (s_param_mutex) xSemaphoreGive(s_param_mutex);
    return ret;
}

esp_err_t cams3_params_reset_defaults(void)
{
    ESP_LOGI(TAG, "Resetting all parameters to factory defaults...");
    if (s_param_mutex) xSemaphoreTake(s_param_mutex, portMAX_DELAY);
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        s_params[i].value = s_params[i].default_val;
        save_param_to_nvs(s_params[i].name, s_params[i].value);
        apply_hardware_setting(s_params[i].name, s_params[i].value);
    }
    if (s_param_mutex) xSemaphoreGive(s_param_mutex);
    return ESP_OK;
}

esp_err_t cams3_params_save_all(void)
{
    if (s_param_mutex) xSemaphoreTake(s_param_mutex, portMAX_DELAY);
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        save_param_to_nvs(s_params[i].name, s_params[i].value);
    }
    if (s_param_mutex) xSemaphoreGive(s_param_mutex);
    return ESP_OK;
}

void cams3_params_register_change_callback(cams3_param_change_cb_t cb)
{
    s_change_cb = cb;
}

// -------------------------------------------------------------
// MAVLink Parameter Protocol Stream Handler
// -------------------------------------------------------------

static void send_param_value_index(uint16_t index)
{
    if (index >= PARAM_COUNT) return;

    mavlink_message_t msg;
    uint8_t packet[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_param_value_pack(
        1,                         // system_id (FC)
        MAV_COMP_ID_OPTICAL_FLOW,  // component_id (197)
        &msg,
        s_params[index].name,
        s_params[index].value,
        s_params[index].type,
        PARAM_COUNT,
        index
    );

    uint16_t len = mavlink_msg_to_send_buffer(packet, &msg);
    mavlink_router_send_internal(packet, len, MAV_ROUTE_DEST_GCS);
}

static void send_all_params_list(void)
{
    ESP_LOGI(TAG, "Responding to PARAM_REQUEST_LIST (%u params total)", (unsigned)PARAM_COUNT);
    for (uint16_t i = 0; i < PARAM_COUNT; i++) {
        send_param_value_index(i);
        // Pacing delay to avoid overloading UDP queue
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}

void cams3_params_handle_mavlink_packet_chan(const uint8_t *buf, uint16_t len, uint8_t chan)
{
    if (!buf || len == 0) return;

    mavlink_message_t msg;
    mavlink_status_t status;

    for (uint16_t i = 0; i < len; i++) {
        if (mavlink_parse_char(chan, buf[i], &msg, &status)) {
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                mavlink_param_request_list_t req;
                mavlink_msg_param_request_list_decode(&msg, &req);
                if ((req.target_system == 1 || req.target_system == 0) &&
                    (req.target_component == MAV_COMP_ID_OPTICAL_FLOW || req.target_component == 0)) {
                    send_all_params_list();
                }
                break;
            }
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                mavlink_param_request_read_t req;
                mavlink_msg_param_request_read_decode(&msg, &req);
                if ((req.target_system == 1 || req.target_system == 0) &&
                    (req.target_component == MAV_COMP_ID_OPTICAL_FLOW || req.target_component == 0)) {
                    if (req.param_index >= 0 && req.param_index < (int16_t)PARAM_COUNT) {
                        send_param_value_index((uint16_t)req.param_index);
                    } else {
                        for (uint16_t idx = 0; idx < PARAM_COUNT; idx++) {
                            if (strncmp(s_params[idx].name, req.param_id, 16) == 0) {
                                send_param_value_index(idx);
                                break;
                            }
                        }
                    }
                }
                break;
            }
            case MAVLINK_MSG_ID_PARAM_SET: {
                mavlink_param_set_t req;
                mavlink_msg_param_set_decode(&msg, &req);
                if ((req.target_system == 1 || req.target_system == 0) &&
                    (req.target_component == MAV_COMP_ID_OPTICAL_FLOW || req.target_component == 0)) {
                    char name_buf[17] = {0};
                    memcpy(name_buf, req.param_id, 16);
                    ESP_LOGI(TAG, "Received PARAM_SET for %s = %.2f from sys %d comp %d",
                             name_buf, req.param_value, msg.sysid, msg.compid);

                    cams3_param_set(name_buf, req.param_value);

                    // Send back updated PARAM_VALUE as confirmation
                    for (uint16_t idx = 0; idx < PARAM_COUNT; idx++) {
                        if (strncmp(s_params[idx].name, name_buf, 16) == 0) {
                            send_param_value_index(idx);
                            break;
                        }
                    }
                }
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_LONG: {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);
                if ((cmd.target_system == 1 || cmd.target_system == 0) &&
                    (cmd.target_component == MAV_COMP_ID_OPTICAL_FLOW || cmd.target_component == 0)) {
                    if (cmd.command == MAV_CMD_PREFLIGHT_STORAGE) {
                        uint8_t ack_res = MAV_RESULT_ACCEPTED;
                        if ((int)cmd.param1 == 0) {
                            cams3_params_init();
                        } else if ((int)cmd.param1 == 1) {
                            cams3_params_save_all();
                        } else if ((int)cmd.param1 == 2) {
                            cams3_params_reset_defaults();
                        } else {
                            ack_res = MAV_RESULT_DENIED;
                        }

                        mavlink_message_t ack_msg;
                        uint8_t ack_pkt[MAVLINK_MAX_PACKET_LEN];
                        mavlink_msg_command_ack_pack(
                            1, MAV_COMP_ID_OPTICAL_FLOW,
                            &ack_msg,
                            cmd.command,
                            ack_res,
                            0, 0,
                            msg.sysid, msg.compid
                        );
                        uint16_t ack_len = mavlink_msg_to_send_buffer(ack_pkt, &ack_msg);
                        mavlink_router_send_internal(ack_pkt, ack_len, MAV_ROUTE_DEST_GCS);
                        ESP_LOGI(TAG, "Processed MAV_CMD_PREFLIGHT_STORAGE (action=%d, result=%d)", (int)cmd.param1, ack_res);
                    }
                }
                break;
            }
            default:
                break;
            }
        }
    }
}

void cams3_params_handle_mavlink_packet(const uint8_t *buf, uint16_t len)
{
    cams3_params_handle_mavlink_packet_chan(buf, len, MAVLINK_COMM_0);
}
