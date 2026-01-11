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
#include "sdkconfig.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "garage.h"
#include "driver/gpio.h"
#include "binary_sensor.h"
#include "relay_driver.h"
#include "ultrasonic_sensor.h"
#include "led_strip.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile garage (End Device) source code.
#endif

#define DOOR_SENSOR "\x0a" \
                    "Door Sensor"

static const char *TAG = "ESP_ZB_GARAGE";
/********************* Define functions **************************/

/* LED state tracking for Identify */
static TaskHandle_t identify_led_task_handle = NULL;
static bool identify_active = false;
static led_strip_handle_t led_strip = NULL;

static sensor_func_pair_t sensor_func_pair[] = {
    {HA_BINARY_SENSOR_ENDPOINT_1, GPIO_NUM_21, SENSOR_TOGGLE_CONTROLL_OFF, GPIO_INPUT_PU_NO},
    {HA_BINARY_SENSOR_ENDPOINT_2, GPIO_NUM_22, SENSOR_TOGGLE_CONTROLL_OFF, GPIO_INPUT_PU_NO}};

static ultrasonic_sensor_func_pair_t ultrasonic_sensor_func_pair[] = {
    {HA_ULTRASONIC_SENSOR_ENDPOINT_1, GPIO_NUM_2, GPIO_NUM_3, ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED}};

static relay_func_pair_t relay_func_pair[] = {
    {HA_RELAY_ENDPOINT, GPIO_NUM_23, NULL}};

/**
 * Initialize the onboard LED for Identify indication
 */
static void identify_led_init(void)
{
    // Configure WS2812 RGB LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = BOARD_LED_GPIO,
        .max_leds = 1, // Single RGB LED
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // Turn off LED initially
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "Identify LED (WS2812) initialized on GPIO %d", BOARD_LED_GPIO);
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
static void zb_identify_notify_handler(uint8_t identify_on)
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

static void zb_binary_sensor_handler(sensor_func_pair_t *sensor_func_pair)
{
    if (esp_zb_bdb_dev_joined())
    {
        // Toggle the binary sensor state
        bool sensor_state = sensor_func_pair->func == SENSOR_TOGGLE_CONTROLL_ON ? true : false;

        ESP_LOGI(TAG, "Sensor changed detected - state is: %s", sensor_state ? "On" : "Off");

        esp_zb_lock_acquire(portMAX_DELAY);

        // Update the attribute value
        ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(sensor_func_pair->endpoint,
                                                     ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                                     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                                                     &sensor_state, false));

        esp_zb_lock_release();

        // Small delay before reporting to ensure attribute is set
        vTaskDelay(pdMS_TO_TICKS(10));

        // Report the attribute change to the coordinator (Home Assistant)
        esp_zb_lock_acquire(portMAX_DELAY);

        esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {
            .zcl_basic_cmd = {
                .src_endpoint = sensor_func_pair->endpoint,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
            .attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        };
        ESP_ERROR_CHECK(esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd));

        esp_zb_lock_release();

        ESP_EARLY_LOGI(TAG, "Binary sensor state: %s", sensor_state ? "On" : "Off");
    }
}

static void zb_ultrasonic_sensor_handler(ultrasonic_sensor_func_pair_t *ultrasonic_sensor_func_pair)
{
    // Check if we're connected to the network before trying to report
    if (esp_zb_bdb_dev_joined())
    {
        bool ultrasonic_state = (ultrasonic_sensor_func_pair->func == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED);

        ESP_LOGI(TAG, "Ultrasonic sensor changed detected - endpoint %d, state is: %s",
                 ultrasonic_sensor_func_pair->endpoint,
                 ultrasonic_state ? "Occupied" : "Unoccupied");

        esp_zb_lock_acquire(portMAX_DELAY);

        uint8_t occupancy_value = ultrasonic_state ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;

        esp_zb_zcl_set_attribute_val(ultrasonic_sensor_func_pair->endpoint,
                                     ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                                     &occupancy_value, false);

        esp_zb_lock_release();

        // Small delay before reporting
        vTaskDelay(pdMS_TO_TICKS(50));

        // Report the attribute change
        esp_zb_lock_acquire(portMAX_DELAY);

        esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {
            .zcl_basic_cmd = {
                .src_endpoint = ultrasonic_sensor_func_pair->endpoint,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            .attributeID = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        };

        esp_err_t err = esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);

        esp_zb_lock_release();

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to report ultrasonic sensor attribute: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "Ultrasonic sensor state reported to coordinator");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Ultrasonic sensor changed but device not joined to network yet, skipping report");
    }
}

static esp_err_t deferred_driver_init(void)
{
    // Initialize LED for Identify functionality
    identify_led_init();

    relay_driver_init(relay_func_pair, RELAY_PAIR_SIZE(relay_func_pair));

    ESP_RETURN_ON_FALSE(binary_sensor_init(sensor_func_pair, SENSOR_PAIR_SIZE(sensor_func_pair), zb_binary_sensor_handler), ESP_FAIL, TAG,
                        "Failed to initialize binary sensor");

    ESP_RETURN_ON_FALSE(ultrasonic_sensor_init(ultrasonic_sensor_func_pair, ULTRASONIC_SENSOR_PAIR_SIZE(ultrasonic_sensor_func_pair), zb_ultrasonic_sensor_handler), ESP_FAIL, TAG,
                        "Failed to initialize ultrasonic sensor");
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

/**
 * The esp_zb_app_signal_handler function is responsible for handling various signals from the Zigbee application layer.
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type)
    {
        /*
        This signal indicates skipping the startup of the Zigbee stack. In this case, we initialize the Zigbee stack and
        then call the esp_zb_bdb_start_top_level_commissioning function to start the top-level commissioning process with the mode
        set to ESP_ZB_BDB_MODE_INITIALIZATION.
        */
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        /*
        These signals indicate the first start or reboot of the device. If the error status is ESP_OK, we perform some initialization tasks,
        such as deferred driver initialization. Then, we check if the device is in the factory new state. If it is, we start the network
        steering process; otherwise, we output a message indicating that the device has rebooted. If the error status is not ESP_OK, we output
        a message indicating that the Zigbee stack initialization failed.
        */
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
        /*
        This signal indicates the status of the network formation process.\
            If the error status is ESP_OK, it retrieves the extended PAN ID, logs information about the formed network (PAN ID, channel, short address),
                and starts the network steering process by calling esp_zb_bdb_start_top_level_commissioning with the mode set to ESP_ZB_BDB_MODE_NETWORK_STEERING.
            If the error status is not ESP_OK, it logs a message to restart the network formation and schedules an alarm to
            call bdb_start_top_level_commissioning_cb after a delay of 1000 milliseconds.
        */
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
        /*
        This signal indicates the result of the network steering process. If the error status is ESP_OK, it means the device successfully joined the network.
        In this case, we output some network information, such as the PAN ID, channel number, and short address. If the error status is not ESP_OK, it means
        the network steering failed, and we output an error message. Then, we use the esp_zb_scheduler_alarm function to set a timer to restart the network
        steering process after a 1-second delay.
        */
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

            // Report initial states after joining network
            binary_sensor_report_initial_states(sensor_func_pair, SENSOR_PAIR_SIZE(sensor_func_pair));
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
        /*
        This signal is triggered when a new device is commissioned or rejoins the network.
            It retrieves the device announcement parameters and logs a message with the short address of the new device.
            It prepares a match descriptor request (esp_zb_zdo_match_desc_req_param_t) with the destination and address of interest set to the new device's short address.
            It calls esp_zb_zdo_find_on_off_light to find a on off light device and specifies user_find_cb as the callback function.
        */
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        esp_zb_zdo_match_desc_req_param_t cmd_req;
        cmd_req.dst_nwk_addr = dev_annce_params->device_short_addr;
        cmd_req.addr_of_interest = dev_annce_params->device_short_addr;
        esp_zb_zdo_find_on_off_light(&cmd_req, relay_find_cb, NULL);
        break;
        /*
        This signal indicates the status of the network's permit join state.
            If the error status is ESP_OK, it logs a message indicating whether the network is open for joining and the duration
                for which it is open. If the network is closed, it logs a warning message.
        */
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK)
        {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p))
            {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            }
            else
            {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
                // Network is now stable and closed - safe to report initial states on reboot
                if (!esp_zb_bdb_is_factory_new())
                {
                    ESP_LOGI(TAG, "Reporting initial sensor states after reboot");
                    binary_sensor_report_initial_states(sensor_func_pair, SENSOR_PAIR_SIZE(sensor_func_pair));
                    ultrasonic_sensor_report_initial_states(ultrasonic_sensor_func_pair, ULTRASONIC_SENSOR_PAIR_SIZE(ultrasonic_sensor_func_pair));
                }
            }
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool relay_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_RELAY_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                relay_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : relay_state;
                ESP_LOGI(TAG, "Relay sets to %s", relay_state ? "On" : "Off");
                relay_driver_set_power(relay_state, message->info.dst_endpoint);
            }
        }
    }
    else if (message->info.dst_endpoint == HA_ULTRASONIC_SENSOR_ENDPOINT_1)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_THRESHOLD_ID &&
                message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8)
            {
                uint8_t threshold = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0;
                ESP_LOGI(TAG, "Ultrasonic detection threshold set to %d cm via Zigbee", threshold);
                ultrasonic_sensor_set_threshold(threshold);
            }
            else if (message->attribute.id == ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY_ID &&
                     message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
            {
                uint16_t delay = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : 0;
                ESP_LOGI(TAG, "Ultrasonic O2U delay set to %d s via Zigbee", delay);
                ultrasonic_sensor_set_o2u_delay(delay);
            }
            else if (message->attribute.id == ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_DELAY_ID &&
                     message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
            {
                uint16_t delay = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : 0;
                ESP_LOGI(TAG, "Ultrasonic U2O delay set to %d s via Zigbee", delay);
                ultrasonic_sensor_set_u2o_delay(delay);
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        // Handle report attribute callback
        ESP_LOGI(TAG, "Report attribute callback received");
        break;
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        // Handle default response callback
        ESP_LOGD(TAG, "Default response callback received");
        break;
    case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
    {
        // Handle identify effect callback
        esp_zb_zcl_identify_effect_message_t *effect_msg = (esp_zb_zcl_identify_effect_message_t *)message;
        ESP_LOGI(TAG, "Identify effect callback: effect_id=%d, effect_variant=%d",
                 effect_msg->effect_id, effect_msg->effect_variant);
        break;
    }
    default:
        ESP_LOGD(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

/**
 * This function defines the endpoint ID, device ID, and the associated clusters.
 * It then initializes the Zigbee stack, registers the device, and starts the main loop.
 */
static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    /*
    Create a single endpoint list for the device to register all endpoints
    Note: Endpoing order in the list defines the order of device signature during network joining process.
    */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    /*
    Endpoint 1 Binary Sensor devices
    */
    static char garage_door_name1[] = "\x0d"
                                      "Garage Door 1";
    esp_zb_binary_sensor_cfg_t sensor_cfg1 = ESP_ZB_DEFAULT_BINARY_SENSOR_CONFIG(HA_BINARY_SENSOR_ENDPOINT_1, garage_door_name1);
    garage_binary_sensor_ep_create(esp_zb_ep_list, &sensor_cfg1);

    /*
    Endpoint 2 Binary Sensor devices
    */
    static char garage_door_name2[] = "\x0d"
                                      "Garage Door 2";
    esp_zb_binary_sensor_cfg_t sensor_cfg2 = ESP_ZB_DEFAULT_BINARY_SENSOR_CONFIG(HA_BINARY_SENSOR_ENDPOINT_2, garage_door_name2);
    garage_binary_sensor_ep_create(esp_zb_ep_list, &sensor_cfg2);

    /*
    Endpoint 3 Ultrasonic Sensor device
    */
    esp_zb_ultrasonic_sensor_cfg_t ultrasonic_cfg = ESP_ZB_DEFAULT_ULTRASONIC_SENSOR_CONFIG(HA_ULTRASONIC_SENSOR_ENDPOINT_1);
    garage_ultrasonic_sensor_ep_create(esp_zb_ep_list, &ultrasonic_cfg);

    /*
    Endpoint 10 for Relay device
    */
    esp_zb_ep_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_EP_ON_OFF_LIGHT_CONFIG(HA_RELAY_ENDPOINT);
    garage_on_off_relay_ep_create(esp_zb_ep_list, &light_cfg);

    // Add manufacturer info to all endpoints
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_RELAY_ENDPOINT, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_BINARY_SENSOR_ENDPOINT_1, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_BINARY_SENSOR_ENDPOINT_2, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_ULTRASONIC_SENSOR_ENDPOINT_1, &info);

    // Register the single device with all endpoints
    esp_zb_device_register(esp_zb_ep_list);

    // Register device and start Zigbee stack
    esp_zb_core_action_handler_register(zb_action_handler);

    // Register identify notification handler for LED indication on each endpoint
    esp_zb_identify_notify_handler_register(HA_RELAY_ENDPOINT, zb_identify_notify_handler);
    esp_zb_identify_notify_handler_register(HA_BINARY_SENSOR_ENDPOINT_1, zb_identify_notify_handler);
    esp_zb_identify_notify_handler_register(HA_BINARY_SENSOR_ENDPOINT_2, zb_identify_notify_handler);
    esp_zb_identify_notify_handler_register(HA_ULTRASONIC_SENSOR_ENDPOINT_1, zb_identify_notify_handler);

    // Set primary channel mask and start the Zigbee stack
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
