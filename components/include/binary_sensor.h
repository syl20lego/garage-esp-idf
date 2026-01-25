/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: LicenseRef-Included
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Espressif Systems
 *    integrated circuit in a product or a software update for such product,
 *    must reproduce the above copyright notice, this list of conditions and
 *    the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * 4. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
        binary_sensor_state_t last_state; // Track per-sensor state to avoid race conditions
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
     * @brief init function for binary sensor and callback setup
     *
     * @param sensor_func_pair      pointer of the button pair.
     * @param sensor_num            number of sensor pair.
     * @param cb                    callback pointer.
     */
    bool binary_sensor_init(sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_sensor_callback_t cb);

    /**
     * @brief Create binary sensor endpoint
     *
     * @param[in] sensor_cfg Configuration for binary sensor
     * @return Endpoint list with binary sensor configuration
     */
    esp_zb_cluster_list_t *garage_binary_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_binary_sensor_cfg_t *sensor_cfg);

    /**
     * @brief Zigbee handler for binary sensor state changes
     *
     * This function handles reporting binary sensor state changes to the Zigbee coordinator.
     * It should be passed as a callback to binary_sensor_init().
     *
     * @param[in] sensor_func_pair Pointer to the sensor that changed state
     */
    void binary_sensor_zb_handler(sensor_func_pair_t *sensor_func_pair);

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