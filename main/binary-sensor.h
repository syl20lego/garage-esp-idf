#pragma once

#include "driver/gpio.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define GPIO_INPUT_IO_TOGGLE_SENSOR GPIO_NUM_22

#define GPIO_INPUT_LEVEL_ON 0

#define ESP_INTR_FLAG_DEFAULT 0

#define PAIR_SIZE(TYPE_STR_PAIR) (sizeof(TYPE_STR_PAIR) / sizeof(TYPE_STR_PAIR[0]))

    typedef enum
    {
        SENSOR_IDLE,
        SENSOR_DETECTED
    } binary_sensor_state_t;

    typedef enum
    {
        SENSOR_TOGGLE_CONTROLL_ON,
        SENSOR_TOGGLE_CONTROLL_OFF
    } sensor_func_t;

    typedef struct
    {
        uint32_t pin;
        sensor_func_t func;
    } sensor_func_pair_t;

    typedef void (*esp_sensor_callback_t)(sensor_func_pair_t *param);

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

    /**
     * @brief init function for binary sensor and callback setup
     *
     * @param sensor_func_pair      pointer of the button pair.
     * @param sensor_num            number of sensor pair.
     * @param cb                    callback pointer.
     */
    bool binary_sensor_init(sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_sensor_callback_t cb);

#ifdef __cplusplus
} // extern "C"
#endif