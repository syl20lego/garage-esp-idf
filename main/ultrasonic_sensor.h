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

#define ULTRASONIC_SENSOR_PAIR_SIZE(TYPE_STR_PAIR) (sizeof(TYPE_STR_PAIR) / sizeof(TYPE_STR_PAIR[0]))

    typedef struct
    {
        uint8_t endpoint;
        gpio_num_t trigger;
        gpio_num_t echo;
        esp_zb_zcl_occupancy_sensing_occupancy_t func;
    } ultrasonic_sensor_func_pair_t;

    typedef void (*esp_ultrasonic_sensor_callback_t)(ultrasonic_sensor_func_pair_t *param);

#define ESP_ZB_DEFAULT_ULTRASONIC_SENSOR_CONFIG(ep)                          \
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
    } esp_zb_ultrasonic_sensor_cfg_t;

    /**
     * @brief Create ultrasonic sensor endpoint
     *
     * @param[in] sensor_cfg Configuration for ultrasonic sensor
     * @return Endpoint list with ultrasonic sensor configuration
     */
    esp_zb_cluster_list_t *garage_ultrasonic_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_ultrasonic_sensor_cfg_t *sensor_cfg);

    /**
     * @brief Report initial ultrasonic sensor states (call after Zigbee network is ready)
     *
     * @param[in] sensor_func_pair Array of sensor configurations
     * @param[in] sensor_num Number of sensors
     */
    void ultrasonic_sensor_report_initial_states(ultrasonic_sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num);

    /**
     * @brief init function for ultrasonic sensor and callback setup
     *
     * @param sensor_func_pair      pointer of the ultrasonic sensor pair.
     * @param sensor_num            number of ultrasonic sensor pair.
     * @param cb                    callback pointer.
     */
    bool ultrasonic_sensor_init(ultrasonic_sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_ultrasonic_sensor_callback_t cb);

    /**
     * @brief Set the ultrasonic detection threshold
     *
     * @param threshold_cm Detection threshold in centimeters (1-254)
     */
    void ultrasonic_sensor_set_threshold(uint8_t threshold_cm);

    /**
     * @brief Get the current ultrasonic detection threshold
     *
     * @return Current threshold in centimeters
     */
    uint8_t ultrasonic_sensor_get_threshold(void);

    /**
     * @brief Set the ultrasonic occupied-to-unoccupied delay
     *
     * @param delay_s Delay in seconds (0-65534)
     */
    void ultrasonic_sensor_set_o2u_delay(uint16_t delay_s);

    /**
     * @brief Get the current ultrasonic occupied-to-unoccupied delay
     *
     * @return Current delay in seconds
     */
    uint16_t ultrasonic_sensor_get_o2u_delay(void);

    /**
     * @brief Set the ultrasonic unoccupied-to-occupied delay
     *
     * @param delay_s Delay in seconds (0-65534)
     */
    void ultrasonic_sensor_set_u2o_delay(uint16_t delay_s);

    /**
     * @brief Get the current ultrasonic unoccupied-to-occupied delay
     *
     * @return Current delay in seconds
     */
    uint16_t ultrasonic_sensor_get_u2o_delay(void);

#ifdef __cplusplus
} // extern "C"
#endif