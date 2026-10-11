#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMS3_MAX_PARAMS 24

// Component ID 197: Visual Inertial Odometry / Optical Flow (ArduPilot convention)
#ifndef MAV_COMP_ID_OPTICAL_FLOW
#define MAV_COMP_ID_OPTICAL_FLOW 197
#endif

// Parameter data types
#define CAMS3_PARAM_TYPE_REAL32 9

/**
 * @brief CamS3 Parameter Definition Structure
 */
typedef struct {
    char     name[16];     // Parameter name (max 16 chars including null)
    float    value;        // Current value
    float    default_val;  // Factory default value
    float    min_val;      // Minimum allowed value
    float    max_val;      // Maximum allowed value
    uint8_t  type;         // Parameter type (e.g. CAMS3_PARAM_TYPE_REAL32)
    bool     readonly;     // True if read-only
} cams3_param_entry_t;

/**
 * @brief Initialize the parameter subsystem.
 * Initializes NVS (Non-Volatile Storage) and loads saved parameters.
 * If a parameter does not exist in NVS, default value is applied and stored.
 *
 * @return ESP_OK on success
 */
esp_err_t cams3_params_init(void);

/**
 * @brief Get parameter value by name
 *
 * @param name Parameter name (e.g. "FLOW_SCALE")
 * @return Parameter value as float, or 0.0f if not found
 */
float cams3_param_get(const char *name);

/**
 * @brief Get parameter value as integer
 *
 * @param name Parameter name
 * @return Integer value
 */
int32_t cams3_param_get_int(const char *name);

/**
 * @brief Set parameter value and persist to NVS
 *
 * @param name Parameter name
 * @param value New value to set
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of bounds or read-only
 */
esp_err_t cams3_param_set(const char *name, float value);

/**
 * @brief Reset all parameters to factory defaults and write to NVS
 *
 * @return ESP_OK on success
 */
esp_err_t cams3_params_reset_defaults(void);

/**
 * @brief Save all current in-memory parameters to NVS
 *
 * @return ESP_OK on success
 */
esp_err_t cams3_params_save_all(void);

/**
 * @brief Callback function type for parameter changes
 */
typedef void (*cams3_param_change_cb_t)(const char *name, float value);

/**
 * @brief Register a callback invoked whenever any parameter changes
 *
 * @param cb Callback function
 */
void cams3_params_register_change_callback(cams3_param_change_cb_t cb);

/**
 * @brief Get total number of registered parameters
 */
uint16_t cams3_params_get_count(void);

/**
 * @brief Get parameter entry by zero-based index
 *
 * @param index Parameter index (0 .. count-1)
 * @param out_entry Pointer to structure receiving the parameter entry
 * @return true if found, false if out of range
 */
bool cams3_params_get_by_index(uint16_t index, cams3_param_entry_t *out_entry);

/**
 * @brief Serialize all parameters into a JSON array string
 *
 * @param buf Output buffer
 * @param max_len Size of output buffer
 * @return ESP_OK on success, ESP_ERR_NO_MEM if buffer is too small
 */
esp_err_t cams3_params_to_json(char *buf, size_t max_len);

#ifdef __cplusplus
}
#endif
