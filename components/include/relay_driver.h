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

#include <stdbool.h>
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* relay state level */
#define RELAY_DEFAULT_ON 1
#define RELAY_DEFAULT_OFF 0

// Macro to create default config with endpoint for On/Off Output device
#define ESP_ZB_DEFAULT_EP_ON_OFF_OUTPUT_CONFIG(ep)                                \
    {                                                                             \
        .basic_cfg =                                                              \
            {                                                                     \
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,        \
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,      \
            },                                                                    \
        .identify_cfg =                                                           \
            {                                                                     \
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE, \
            },                                                                    \
        .on_off_cfg =                                                             \
            {                                                                     \
                .on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE,                 \
            },                                                                    \
        .endpoint = ep,                                                           \
    }

    typedef struct relay_func_pair_s
    {
        uint8_t endpoint;
        gpio_num_t gpio;
        TaskHandle_t pulse_task_handle;
        uint16_t pulse_duration_ms;
    } relay_func_pair_t;

    // Extended config struct for On/Off Output device
    typedef struct
    {
        esp_zb_basic_cluster_cfg_t basic_cfg;
        esp_zb_identify_cluster_cfg_t identify_cfg;
        esp_zb_on_off_cluster_cfg_t on_off_cfg;
        uint8_t endpoint;
    } esp_zb_ep_on_off_output_cfg_t;

    /**
     * @brief Early GPIO initialization for relay pins to prevent false activation on boot.
     *        Call this as early as possible in app_main, before any other initialization.
     *        This sets relay GPIO pins HIGH (relay OFF) before the full driver init.
     *
     * @param gpio_pin The GPIO pin number for the relay
     */
    void relay_driver_early_init(gpio_num_t gpio_pin);

    /**
     * @brief Initialize relay driver with multiple relays
     * @param relay_pairs Array of relay configurations
     * @param relay_pair_count Number of relays in the array
     */
    void relay_driver_init(relay_func_pair_t *relay_pairs, uint8_t relay_pair_count);

    /**
     * @brief Create on/off output endpoint with configuration
     *
     * @param[in] ep_list Endpoint list to add the output endpoint to
     * @param[in] ep_output_cfg Extended configuration including endpoint ID
     * @return Cluster list for the output endpoint
     */
    esp_zb_cluster_list_t *garage_on_off_relay_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_ep_on_off_output_cfg_t *ep_output_cfg);

    /**
     * @brief Callback when a relay device is found
     *
     * @param[in] zdo_status Status of the find operation
     * @param[in] addr Short address of the found device
     * @param[in] endpoint Endpoint of the found device
     * @param[in] user_ctx User context (unused)
     */
    void relay_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx);

    /**
     * @brief Set relay power (on/off) with automatic pulse handling.
     *        When turned on, relay will pulse for configured duration then turn off.
     *
     * @param  power    The relay power to be set (true = pulse, false = force off)
     * @param  endpoint The endpoint ID for reporting status back to coordinator
     */
    void relay_driver_set_power(bool power, uint8_t endpoint);

    /**
     * @brief Load relay settings from NVS (call during initialization)
     */
    void relay_driver_load_settings(void);

    /**
     * @brief Get the pulse duration for a specific relay endpoint
     * @param endpoint The endpoint ID
     * @return Pulse duration in milliseconds (or default if endpoint not found)
     */
    uint16_t relay_driver_get_pulse_duration_ms(uint8_t endpoint);

    /**
     * @brief Set the pulse duration for a specific relay endpoint
     * @param endpoint The endpoint ID
     * @param duration_ms Pulse duration in milliseconds (min 100, max 30000)
     */
    void relay_driver_set_pulse_duration_ms(uint8_t endpoint, uint16_t duration_ms);

    /**
     * @brief Set the relay pulse duration from Zigbee OnTime attribute
     * @param endpoint The endpoint ID
     * @param on_time_tenth_seconds OnTime value in 1/10th of a second units
     */
    void relay_driver_set_pulse_duration_from_ontime(uint8_t endpoint, uint16_t on_time_tenth_seconds);

#ifdef __cplusplus
} // extern "C"
#endif
