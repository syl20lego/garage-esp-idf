#pragma once

#include "driver/gpio.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ESP_ZB_HA_OCCUPANCY_SENSOR_DEVICE_ID 0x0107

#define OCCUPENCY_PAIR_SIZE(TYPE_STR_PAIR) (sizeof(TYPE_STR_PAIR) / sizeof(TYPE_STR_PAIR[0]))

    typedef enum
    {
        OCCUPENCY_IDLE,
        OCCUPENCY_DETECTED
    } occupency_sensor_state_t;

    typedef enum
    {
        OCCUPENCY_TOGGLE_CONTROLL_OFF = 0,
        OCCUPENCY_TOGGLE_CONTROLL_ON = 1
    } occupency_func_t;

    typedef struct
    {
        gpio_num_t pin;
        occupency_func_t func;
        bool normal_level;
    } occupency_func_pair_t;

    typedef void (*esp_occupency_callback_t)(occupency_func_pair_t *param);

#define ESP_ZB_DEFAULT_OCCCUPENCY_SENSOR_CONFIG(ep)                          \
    {                                                                        \
        .basic_cfg =                                                         \
            {                                                                \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,   \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE, \
            },                                                               \
        .endpoint = ep,                                                      \
    }

    // Define the config struct
    typedef struct
    {
        esp_zb_basic_cluster_cfg_t basic_cfg;
        uint8_t endpoint;
        char *sensor_name; // This field name must match what's in the macro
    } esp_zb_occupency_sensor_cfg_t;

    /**
     * @brief Create occupency sensor endpoint
     *
     * @param[in] occupency_cfg Configuration for occupency sensor
     * @return Endpoint list with occupency sensor configuration
     */
    esp_zb_cluster_list_t *garage_occupency_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_occupency_sensor_cfg_t *sensor_cfg);

    /**
     * @brief init function for occupency sensor and callback setup
     *
     * @param sensor_func_pair      pointer of the button pair.
     * @param sensor_num            number of sensor pair.
     * @param cb                    callback pointer.
     */
    bool occupency_sensor_init(occupency_func_pair_t *occupency_func_pair, uint8_t sensor_num, esp_occupency_callback_t cb);

#ifdef __cplusplus
} // extern "C"
#endif