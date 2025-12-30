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

/* light intensity level */
#define LIGHT_DEFAULT_ON 1
#define LIGHT_DEFAULT_OFF 0

/* LED strip configuration */
#define CONFIG_EXAMPLE_STRIP_LED_GPIO 8
#define CONFIG_EXAMPLE_STRIP_LED_NUMBER 1

    // Extended config struct that includes endpoint configuration
    typedef struct
    {
        esp_zb_on_off_light_cfg_t light_cfg;
        uint8_t endpoint;
    } esp_zb_ep_on_off_light_cfg_t;

// Macro to create default config with endpoint
#define ESP_ZB_DEFAULT_EP_ON_OFF_LIGHT_CONFIG(ep)          \
    {                                                      \
        .light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG(), \
        .endpoint = ep,                                    \
    }

    /**
     * @brief Set light power (on/off).
     *
     * @param  power  The light power to be set
     */
    void light_driver_set_power(bool power);

    /**
     * @brief color light driver init, be invoked where you want to use color light
     *
     * @param power power on/off
     */
    void light_driver_init(bool power);

    /**
     * @brief Create on/off light endpoint with configuration
     *
     * @param[in] ep_list Endpoint list to add the light endpoint to
     * @param[in] ep_light_cfg Extended configuration including endpoint ID
     * @return Cluster list for the light endpoint
     */
    esp_zb_cluster_list_t *garage_on_off_light_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_ep_on_off_light_cfg_t *ep_light_cfg);

#ifdef __cplusplus
} // extern "C"
#endif
