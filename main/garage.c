/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier:  LicenseRef-Included
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
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
#include "motor_driver.h"
#include "binary-sensor.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile garage (End Device) source code.
#endif

#define DOOR_SENSOR "\x0a" \
                    "Door Sensor"

static const char *TAG = "ESP_ZB_GARAGE";
/********************* Define functions **************************/
typedef struct light_bulb_device_params_s
{
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

static sensor_func_pair_t sensor_func_pair[] = {
    {HA_BINARY_SENSOR_ENDPOINT_1, GPIO_NUM_21, SENSOR_TOGGLE_CONTROLL_OFF, GPIO_INPUT_PU_NO},
    {HA_BINARY_SENSOR_ENDPOINT_1, GPIO_NUM_21, SENSOR_TOGGLE_CONTROLL_ON, GPIO_INPUT_PU_NO},
    {HA_BINARY_SENSOR_ENDPOINT_2, GPIO_NUM_22, SENSOR_TOGGLE_CONTROLL_OFF, GPIO_INPUT_PU_NO},
    {HA_BINARY_SENSOR_ENDPOINT_2, GPIO_NUM_22, SENSOR_TOGGLE_CONTROLL_ON, GPIO_INPUT_PU_NO}};

static void zb_sensor_handler(sensor_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SENSOR_TOGGLE_CONTROLL_ON ||
        button_func_pair->func == SENSOR_TOGGLE_CONTROLL_OFF)
    {
        // Toggle the binary sensor state
        bool sensor_state = button_func_pair->func == SENSOR_TOGGLE_CONTROLL_ON ? true : false;

        ESP_LOGI(TAG, "Sensor changed detected - state is: %s", sensor_state ? "On" : "Off");

        esp_zb_lock_acquire(portMAX_DELAY);

        // Update the attribute value
        ESP_ERROR_CHECK(esp_zb_zcl_set_attribute_val(button_func_pair->endpoint,
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
                .src_endpoint = button_func_pair->endpoint,
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

static esp_err_t deferred_driver_init(void)
{
    motor_driver_init(MOTOR_DEFAULT_OFF);
    ESP_RETURN_ON_FALSE(binary_sensor_init(sensor_func_pair, SENSOR_PAIR_SIZE(sensor_func_pair), zb_sensor_handler), ESP_FAIL, TAG,
                        "Failed to initialize binary sensor");
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS)
    {
        ESP_LOGI(TAG, "Bound successfully!");
        if (user_ctx)
        {
            light_bulb_device_params_t *light = (light_bulb_device_params_t *)user_ctx;
            ESP_LOGI(TAG, "The light originating from address(0x%x) on endpoint(%d)", light->short_addr, light->endpoint);
            free(light);
        }
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS)
    {
        ESP_LOGI(TAG, "Found light");
        esp_zb_zdo_bind_req_param_t bind_req;
        light_bulb_device_params_t *light = (light_bulb_device_params_t *)malloc(sizeof(light_bulb_device_params_t));
        light->endpoint = endpoint;
        light->short_addr = addr;
        esp_zb_ieee_address_by_short(light->short_addr, light->ieee_addr);
        esp_zb_get_long_address(bind_req.src_address);
        bind_req.src_endp = HA_RELAY_SENSOR_ENDPOINT_1;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req.dst_address_u.addr_long, light->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req.dst_endp = endpoint;
        bind_req.req_dst_addr = esp_zb_get_short_address();
        ESP_LOGI(TAG, "Try to bind On/Off");
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *)light);
    }
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
        esp_zb_zdo_find_on_off_light(&cmd_req, user_find_cb, NULL);
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
    bool motor_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT)
    {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
        {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
            {
                motor_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : motor_state;
                ESP_LOGI(TAG, "Motor sets to %s", motor_state ? "On" : "Off");
                motor_driver_set_power(motor_state);
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

    // Create a single endpoint list with both endpoints
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();

    esp_zb_ep_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_EP_ON_OFF_LIGHT_CONFIG(HA_ESP_LIGHT_ENDPOINT);
    garage_on_off_motor_ep_create(esp_zb_ep_list, &light_cfg);

    // Create binary sensor endpoint
    static char garage_door_name1[] = "\x0d"
                                      "Garage Door 1";
    esp_zb_binary_sensor_cfg_t sensor_cfg1 = ESP_ZB_DEFAULT_BINARY_SENSOR_CONFIG(HA_BINARY_SENSOR_ENDPOINT_1, garage_door_name1);
    garage_binary_sensor_ep_create(esp_zb_ep_list, &sensor_cfg1);

    // Create binary sensor endpoint
    static char garage_door_name2[] = "\x0d"
                                      "Garage Door 2";
    esp_zb_binary_sensor_cfg_t sensor_cfg2 = ESP_ZB_DEFAULT_BINARY_SENSOR_CONFIG(HA_BINARY_SENSOR_ENDPOINT_2, garage_door_name2);
    garage_binary_sensor_ep_create(esp_zb_ep_list, &sensor_cfg2);

    // Add manufacturer info to both endpoints
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_ESP_LIGHT_ENDPOINT, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_BINARY_SENSOR_ENDPOINT_1, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list, HA_BINARY_SENSOR_ENDPOINT_2, &info);

    // Register the single device with both endpoints
    esp_zb_device_register(esp_zb_ep_list);

    // Register device and start Zigbee stack
    esp_zb_core_action_handler_register(zb_action_handler);
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
