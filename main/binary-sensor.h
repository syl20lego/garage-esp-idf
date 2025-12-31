#pragma once

#include "driver/gpio.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Pull UP Normaly Closed or Open */
#define GPIO_INPUT_PU_NO 0
#define GPIO_INPUT__PU_NC 1

#define SENSOR_PAIR_SIZE(TYPE_STR_PAIR) (sizeof(TYPE_STR_PAIR) / sizeof(TYPE_STR_PAIR[0]))

    typedef enum
    {
        SENSOR_IDLE,
        SENSOR_DETECTED
    } binary_sensor_state_t;

    typedef enum
    {
        SENSOR_TOGGLE_CONTROLL_OFF = 0,
        SENSOR_TOGGLE_CONTROLL_ON = 1
    } sensor_func_t;

    typedef struct
    {
        uint8_t endpoint;
        gpio_num_t pin;
        sensor_func_t func;
        bool normal_level;
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

    /**
     * @brief Report initial sensor states (call after Zigbee network is ready)
     *
     * @param[in] sensor_func_pair Array of sensor configurations
     * @param[in] sensor_num Number of sensors
     */
    void binary_sensor_report_initial_states(sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num);

#ifdef __cplusplus
} // extern "C"
#endif