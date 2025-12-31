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
        uint8_t endpoint;
        gpio_num_t trigger;
        gpio_num_t echo;
        occupency_func_t func;
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
     * @brief Report initial occupancy sensor states (call after Zigbee network is ready)
     *
     * @param[in] sensor_func_pair Array of sensor configurations
     * @param[in] sensor_num Number of sensors
     */
    void occupency_sensor_report_initial_states(occupency_func_pair_t *sensor_func_pair, uint8_t sensor_num);

    /**
     * @brief init function for occupency sensor and callback setup
     *
     * @param sensor_func_pair      pointer of the occupency pair.
     * @param sensor_num            number of occupency pair.
     * @param cb                    callback pointer.
     */
    bool occupency_sensor_init(occupency_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_occupency_callback_t cb);

#ifdef __cplusplus
} // extern "C"
#endif