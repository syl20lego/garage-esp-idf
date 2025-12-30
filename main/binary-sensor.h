#pragma once

#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#define ESP_ZB_DEFAULT_BINARY_SENSOR_CONFIG(name)                            \
    {                                                                        \
        .basic_cfg =                                                         \
            {                                                                \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,   \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE, \
            },                                                               \
        .sensor_name = name,                                                 \
    }

// Define the config struct
typedef struct
{
    esp_zb_basic_cluster_cfg_t basic_cfg;
    char *sensor_name; // This field name must match what's in the macro
} esp_zb_binary_sensor_cfg_t;

/**
 * @brief Create binary sensor endpoint
 *
 * @param[in] endpoint Endpoint ID for the binary sensor
 * @param[in] sensor_cfg Configuration for binary sensor (includes name)
 * @return Endpoint list with binary sensor configuration
 */
esp_zb_ep_list_t *esp_zb_binary_sensor_ep_create(uint8_t endpoint, esp_zb_binary_sensor_cfg_t *sensor_cfg);