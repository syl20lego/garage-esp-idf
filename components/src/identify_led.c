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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "identify_led.h"

static const char *TAG = "IDENTIFY_LED";

/* LED state tracking for Identify */
static TaskHandle_t identify_led_task_handle = NULL;
static bool identify_active = false;
static led_strip_handle_t led_strip = NULL;

/**
 * Initialize the onboard LED for Identify indication
 */
void identify_led_init(void)
{
    // Configure WS2812 RGB LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = IDENTIFY_LED_GPIO,
        .max_leds = 1, // Single RGB LED
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Turn off LED initially
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "Identify LED (WS2812) initialized on GPIO %d", IDENTIFY_LED_GPIO);
}

/**
 * Task to blink LED during identify period
 */
static void identify_led_blink_task(void *pvParameters)
{
    uint16_t identify_time = *(uint16_t *)pvParameters;
    ESP_LOGI(TAG, "Starting identify LED blink for %d seconds", identify_time);

    identify_active = true;
    uint32_t end_time = xTaskGetTickCount() + pdMS_TO_TICKS(identify_time * 1000);

    while (xTaskGetTickCount() < end_time && identify_active)
    {
        // LED ON - White color
        led_strip_set_pixel(led_strip, 0, 25, 25, 25); // White at low brightness
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(IDENTIFY_BLINK_PERIOD_MS));

        // LED OFF
        led_strip_clear(led_strip);
        vTaskDelay(pdMS_TO_TICKS(IDENTIFY_BLINK_PERIOD_MS));
    }

    // Ensure LED is off when done
    led_strip_clear(led_strip);
    identify_active = false;
    identify_led_task_handle = NULL;
    ESP_LOGI(TAG, "Identify LED blink completed");
    vTaskDelete(NULL);
}

/**
 * Zigbee Identify callback - called when identify command is received
 */
void identify_led_notify_handler(uint8_t identify_on)
{
    ESP_LOGI(TAG, "Identify command received, identify_on: %d", identify_on);

    if (identify_on)
    {
        // Start LED blinking - default to 5 seconds if no time is specified
        static uint16_t time_param = 5;

        // Stop any existing identify task
        if (identify_led_task_handle != NULL)
        {
            identify_active = false;
            vTaskDelay(pdMS_TO_TICKS(50)); // Give time for task to exit
            if (identify_led_task_handle != NULL)
            {
                vTaskDelete(identify_led_task_handle);
                identify_led_task_handle = NULL;
            }
        }

        xTaskCreate(identify_led_blink_task, "identify_led", 2048, &time_param, 5, &identify_led_task_handle);
    }
    else
    {
        // Stop identifying - turn off LED
        if (identify_led_task_handle != NULL)
        {
            identify_active = false;
            vTaskDelay(pdMS_TO_TICKS(50));
            if (identify_led_task_handle != NULL)
            {
                vTaskDelete(identify_led_task_handle);
                identify_led_task_handle = NULL;
            }
        }
        led_strip_clear(led_strip); // LED off
    }
}
