#pragma once

#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ESP_ZB_DEFAULT_BINARY_SENSOR_CONFIG(ep, name)                        \
    {                                                                        \
        .basic_cfg =                                                         \
            {                                                                \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,   \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE, \
            },                                                               \
        .endpoint = ep,                                                      \
        .sensor_name = name,                                                 \
    }

    // Define the config struct
    typedef struct
    {
        esp_zb_basic_cluster_cfg_t basic_cfg;
        uint8_t endpoint;
        char *sensor_name; // This field name must match what's in the macro
    } esp_zb_binary_sensor_cfg_t;

    /**
     * @brief Create binary sensor endpoint
     *
     * @param[in] sensor_cfg Configuration for binary sensor
     * @return Endpoint list with binary sensor configuration
     */
    esp_zb_cluster_list_t *garage_binary_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_binary_sensor_cfg_t *sensor_cfg);

#ifdef __cplusplus
} // extern "C"
#endif