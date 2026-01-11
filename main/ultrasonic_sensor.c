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
#include "string.h"
#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic_sensor.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "nvs.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define ULTRASONIC_MAX_TIMEOUT_US 30000              // 30ms timeout (increased from 25ms)
#define ULTRASONIC_TRIGGER_PULSE_US 50               // 50us trigger pulse
#define ULTRASONIC_DETECTION_DISTANCE_CM_DEFAULT 100 // Default detection distance in cm
#define ULTRASONIC_MAX_DISTANCE_CM 400               // Maximum measurable distance (400cm = ~23ms round trip)

#define NVS_NAMESPACE "occupancy"
#define NVS_KEY_THRESHOLD "threshold"
#define NVS_KEY_O2U_DELAY "o2u_delay"
#define NVS_KEY_U2O_DELAY "u2o_delay"

#define ULTRASONIC_O2U_DELAY_DEFAULT 0 // Default occupied-to-unoccupied delay in seconds
#define ULTRASONIC_U2O_DELAY_DEFAULT 0 // Default unoccupied-to-occupied delay in seconds

/* ultrasonic sensor function pair, should be defined in switch example source file */
static ultrasonic_sensor_func_pair_t *ultrasonic_sensor_func_pair;
/* call back function pointer */
static esp_ultrasonic_sensor_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
/* Ultrasonic detection threshold in cm (configurable via Zigbee) */
static uint8_t ultrasonic_detection_threshold_cm = ULTRASONIC_DETECTION_DISTANCE_CM_DEFAULT;
/* Ultrasonic occupied-to-unoccupied delay in seconds (configurable via Zigbee) */
static uint16_t ultrasonic_o2u_delay_s = ULTRASONIC_O2U_DELAY_DEFAULT;
/* Ultrasonic unoccupied-to-occupied delay in seconds (configurable via Zigbee) */
static uint16_t ultrasonic_u2o_delay_s = ULTRASONIC_U2O_DELAY_DEFAULT;
static const char *TAG = "ESP_ZB_ULTRASONIC";

/**
    4.8 Occupancy Sensing
    Please see Chapter 2 for a general cluster overview defining cluster architecture, revision, classification,
    identification, etc.
    The server cluster provides an interface to occupancy sensing functionality, including configuration and
    provision of notifications of occupancy status.

    @param[in] sensor_cfg Configuration for ultrasonic sensor
    @return Endpoint list with ultrasonic sensor configuration
*/
esp_zb_cluster_list_t *garage_ultrasonic_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_ultrasonic_sensor_cfg_t *sensor_cfg)
{
    // Load settings from NVS BEFORE creating the Zigbee attributes
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        uint8_t saved_threshold = ULTRASONIC_DETECTION_DISTANCE_CM_DEFAULT;
        err = nvs_get_u8(nvs_handle, NVS_KEY_THRESHOLD, &saved_threshold);
        if (err == ESP_OK)
        {
            ultrasonic_detection_threshold_cm = saved_threshold;
            ESP_LOGI(TAG, "Loaded threshold from NVS: %d cm", ultrasonic_detection_threshold_cm);
        }

        uint16_t saved_o2u_delay = ULTRASONIC_O2U_DELAY_DEFAULT;
        err = nvs_get_u16(nvs_handle, NVS_KEY_O2U_DELAY, &saved_o2u_delay);
        if (err == ESP_OK)
        {
            ultrasonic_o2u_delay_s = saved_o2u_delay;
            ESP_LOGI(TAG, "Loaded O2U delay from NVS: %d s", ultrasonic_o2u_delay_s);
        }

        uint16_t saved_u2o_delay = ULTRASONIC_U2O_DELAY_DEFAULT;
        err = nvs_get_u16(nvs_handle, NVS_KEY_U2O_DELAY, &saved_u2o_delay);
        if (err == ESP_OK)
        {
            ultrasonic_u2o_delay_s = saved_u2o_delay;
            ESP_LOGI(TAG, "Loaded U2O delay from NVS: %d s", ultrasonic_u2o_delay_s);
        }

        nvs_close(nvs_handle);
    }

    // Create cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&sensor_cfg->basic_cfg);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*
    https://zigbeealliance.org/wp-content/uploads/2021/10/07-5123-08-Zigbee-Cluster-Library.pdf

    Occupancy Sensing cluster
    4.8.1.3 Cluster Identifiers

    Identifier Name
    0x0406 Occupancy Sensing

    4.8.2.2 Attributes

    For convenience, the attributes defined in this specification are arranged into sets of related attributes; each
    set can contain up to 16 attributes. Attribute identifiers are encoded such that the most significant
    three nibbles specify the attribute set and the least significant nibble specifies the attribute within the set.
    The currently defined attribute sets are listed in Table

    Attribute Set Identifier        Description
    0x000                           Occupancy sensor information
    0x001                           PIR configuration
    0x002                           Ultrasonic configuration

    4.8.2.2.1 Occupancy Sensor Information Set

    The occupancy sensor information attribute set contains the attributes summarized in Table

    Attribute ID   Name                 Type        Range               Access  Default         M/O
    0x0000      Occupancy               map8        0b0000 000x         RP      -               M
    0x0001      OccupancySensorType     enum8       0x00 – 0xfe         R       -               M

    */
    esp_zb_attribute_list_t *input_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);
    // Occupancy attribute (map8, 0x0000)
    uint8_t occupancy = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
    esp_zb_occupancy_sensing_cluster_add_attr(input_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy);

    /*
     4.8.2.2.1.2 OccupancySensorType Attribute
        The OccupancySensorType attribute specifies the type of the occupancy sensor. This attribute shall be set to
        one of the non-reserved values listed in Table 4-22.
        Attribute Value     Description
        0x00                PIR
        0x01                Ultrasonic
        0x02                PIR and ultrasonic
        0x03                Physical contact
    */
    // Occupancy Sensor Type attribute (enum8, 0x0001) - needs to be a pointer
    uint8_t sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC;
    esp_zb_occupancy_sensing_cluster_add_attr(input_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &sensor_type);

    /*
    4.8.2.2.1.3 OccupancySensorTypeBitmap Attribute
        The OccupancySensorTypeBitmap attribute specifies the types of the occupancy sensor, as listed below; a
        ‘1’ in each bit position indicates this type is implemented.
        Bit Description
        Bit0 PIR
        Bit1 Ultrasonic
        Bit2 Physical contact
    */
    // Occupancy Sensor Type Bitmap attribute for Ultrasonic 0000 0010
    // uint8_t sensor_type_bitmap = 0x02;
    // esp_zb_occupancy_sensing_cluster_add_attr(input_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_BITMAP_ID, &sensor_type_bitmap);

    /*
    4.8.2.2.3 Ultrasonic Configuration Set
        The ultrasonic sensor configuration attribute set contains the attributes summarized in Table 4-26
        Id      Name                                    Type    Range           Acc     Def     MO
        0x0020  UltrasonicOccupiedToUnoccupiedDelay     uint16  0x0000 – 0xfffe RW      0x00    O
        0x0021  UltrasonicUnoccupiedToOccupiedDelay     uint16  0x0000 – 0xfffe RW      0x00    O
        0x0022  UltrasonicUnoccupiedToOccupiedThreshold uint8   0x01 – 0xfe     RW      0x01    O
    */

    /*
    4.8.2.2.3.1 UltrasonicOccupiedToUnoccupiedDelay Attribute
        The UltrasonicOccupiedToUnoccupiedDelay attribute is 16 bits in length and specifies the time delay, in
        seconds, before the Ultrasonic sensor changes to its unoccupied state after the last detection of movement in
        the sensed area.
    */
    // UltrasonicOccupiedToUnoccupiedDelay attribute (uint16, 0x0020) - configurable via HA
    // This represents the delay in seconds before transitioning from occupied to unoccupied
    uint16_t o2u_delay = ultrasonic_o2u_delay_s;
    esp_zb_cluster_add_attr(input_cluster,
                            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                            &o2u_delay);

    /*
    4.8.2.2.3.2 UltrasonicUnoccupiedToOccupiedDelay Attribute
        The UltrasonicUnoccupiedToOccupiedDelay attribute is 16 bits in length and specifies the time delay, in
        seconds, before the Ultrasonic sensor changes to its occupied state after the detection of movement in the
        sensed area. This attribute is mandatory if the UltrasonicUnoccupiedToOccupiedThreshold attribute is im8073 plemented.
    */
    // UltrasonicUnoccupiedToOccupiedDelay attribute (uint16, 0x0021) - configurable via HA
    // This represents the delay in seconds before transitioning from unoccupied to occupied
    uint16_t u2o_delay = ultrasonic_u2o_delay_s;
    esp_zb_cluster_add_attr(input_cluster,
                            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_DELAY_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U16,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                            &u2o_delay);

    /*
    UltrasonicUnoccupiedToOccupiedThreshold Attribute
        The UltrasonicUnoccupiedToOccupiedThreshold attribute is 8 bits in length and specifies the number of
        movement detection events that must occur in the period UltrasonicUnoccupiedToOccupiedDelay, before
        the Ultrasonic sensor changes to its occupied state. This attribute is mandatory if the UltrasonicUnoccu8078 piedToOccupiedDelay attribute is implemented.
    */
    // UltrasonicUnoccupiedToOccupiedThreshold attribute (uint8, 0x0022) - configurable via HA
    // This represents the detection distance threshold in cm (1-254)
    // Use the value loaded from NVS (or default if not saved)
    uint8_t threshold = ultrasonic_detection_threshold_cm;
    esp_zb_cluster_add_attr(input_cluster,
                            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_UNOCCUPIED_TO_OCCUPIED_THRESHOLD_ID,
                            ESP_ZB_ZCL_ATTR_TYPE_U8,
                            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
                            &threshold);

    esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, input_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*

    Create endpoint configuration
    When configuring an endpoint, you must specify a Profile ID. If you are developing a Zigbee 3.0 application,
    you must specify the profile ID 0x0104.

    https://zigbeealliance.org/wp-content/uploads/2019/11/docs-05-3474-21-0csg-zigbee-specification.pdf
    2.3.2.5 Simple Descriptor

    The simple descriptor contains information specific to each endpoint contained in this node. The simple
    descriptor is mandatory for each endpoint present in the node.
    The fields of the simple descriptor are shown in Table 2.39 in their order of transmission. As this descriptor
    needs to be transmitted over air, the overall length of the simple descriptor shall be less than or equal to
    apscMaxDescriptorSize.

    Field Name                          Length (Bits)
    Endpoint                            8
    Application profile identifier      16
    Application device identifier       16
    Application device version          4
    Reserved                            4
    Application input cluster count     8
    Application input cluster list      16*i (where i is the value of the application input cluster count)
    Application output cluster count    8
    Application output cluster list     16*o (where o is the value of the application output cluster count)

    https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/wireless-connectivity/698/1/075367r03ZB_AFG-Home_Automation_Profile_for_Public_Download.pdf

    Table 5.1 Devices Specified in the HA Profile
    Device                          Device ID
    ======================
    Generic
    ======================
    On/Off Switch                           0x0000
    Level Control Switch                    0x0001
    On/Off Output                           0x0002
    Level Controllable Output               0x0003
    Scene Selector                          0x0004
    Configuration Tool                      0x0005
    Remote Control                          0x0006
    Combined Interface                      0x0007
    Range Extender                          0x0008
    Mains Power Outlet                      0x0009
    Door Lock                               0x000A
    Door Lock Controller                    0x000B
    Simple Sensor                           0x000C
    Consumption Awareness Device            0x000D
    Home Gateway                            0x0050
    Smart plug                              0x0051
    White Goods                             0x0052
    Meter Interface                         0x0053
    Reserved                                0x0060– 0x00FF
    ======================
    Lighting
    ======================
    On/Off Light                            0x0100
    Dimmable Light                          0x0101
    Color Dimmable Light                    0x0102
    On/Off Light Switch                     0x0103
    Dimmer Switch                           0x0104
    Color Dimmer Switch                     0x0105
    Light Sensor                            0x0106
    Occupancy Sensor                        0x0107
    Reserved                                0x0108 – 0x1FF
    ======================
    Closures
    ======================
    Shade                                   0x0200
    Shade Controller                        0x0201
    Window Covering Device                  0x0202
    Window Covering Controller              0x0203
    Reserved                                0x0204 – 0x2FF
    ======================
    HVAC
    ======================
    Heating/Cooling Unit                    0x0300
    Thermostat                              0x0301
    Temperature Sensor                      0x0302
    Pump                                    0x0303
    Pump Controller                         0x0304
    Pressure Sensor                         0x0305
    Flow Sensor                             0x0306
    Mini Split AC                           0x0307
    Reserved                                0x0308 - 0x3FF
    ======================
    Intruder Alarm Systems
    ======================
    IAS Control and Indicating Equipment    0x0400
    IAS Ancillary Control Equipment         0x0401
    IAS Zone                                0x0402
    IAS Warning Device                      0x0403
    Reserved                                0x0404-0xFFFF

    */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = sensor_cfg->endpoint,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_OCCUPANCY_SENSOR_DEVICE_ID,
        .app_device_version = 0};

    // Add enpoint to the endpoint list
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    return cluster_list;
}

/**
 * @brief Measure distance using HC-SR04 ultrasonic sensor
 *
 * @param trigger_pin GPIO pin connected to trigger
 * @param echo_pin GPIO pin connected to echo
 * @return Distance in centimeters, or -1 if timeout/error
 */
static int32_t ultrasonic_measure_distance(gpio_num_t trigger_pin, gpio_num_t echo_pin)
{
    // Ensure trigger is LOW initially
    gpio_set_level(trigger_pin, 0);
    esp_rom_delay_us(5);

    // Send 10us trigger pulse
    gpio_set_level(trigger_pin, 1);
    esp_rom_delay_us(ULTRASONIC_TRIGGER_PULSE_US);
    gpio_set_level(trigger_pin, 0);

    // Wait for echo pin to go HIGH (with timeout)
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0)
    {
        if ((esp_timer_get_time() - start_time) > 10000) // 10ms timeout for echo start
        {
            ESP_LOGD(TAG, "Timeout waiting for echo pin to go HIGH");
            return -1;
        }
    }

    // Measure how long echo pin stays HIGH
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1)
    {
        int64_t elapsed = esp_timer_get_time() - pulse_start;
        if (elapsed > ULTRASONIC_MAX_TIMEOUT_US)
        {
            ESP_LOGD(TAG, "Echo pulse too long (>30ms), object out of range");
            return ULTRASONIC_MAX_DISTANCE_CM + 1; // Return value indicating out of range
        }
    }
    int64_t pulse_end = esp_timer_get_time();

    // Calculate distance in cm
    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (time * speed) / 2 (round trip)
    int64_t pulse_duration_us = pulse_end - pulse_start;
    int32_t distance_cm = (pulse_duration_us * 343) / (2 * 10000);

    // Sanity check
    if (distance_cm < 2 || distance_cm > ULTRASONIC_MAX_DISTANCE_CM)
    {
        ESP_LOGD(TAG, "Distance out of valid range: %ld cm", distance_cm);
        return -1;
    }

    return distance_cm;
}

/**
 * @brief Task for periodically checking ultrasonic sensor
 */
static void ultrasonic_sensor_task(void *arg)
{
    static esp_zb_zcl_occupancy_sensing_occupancy_t previous_state[10] = {ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED}; // Support up to 10 sensors
    static esp_zb_zcl_occupancy_sensing_occupancy_t raw_state[10] = {ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED};      // Raw detected state before delay
    static int64_t state_change_time[10] = {0};                                                                               // Time when raw state changed (in ms)
    static int consecutive_errors[10] = {0};                                                                                  // Track consecutive errors per sensor
    static bool task_ready = false;

    // Wait for Zigbee stack to initialize
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Starting ultrasonic sensor polling task");
    task_ready = true;

    while (1)
    {
        int64_t current_time_ms = esp_timer_get_time() / 1000; // Current time in milliseconds

        for (int i = 0; i < switch_num; i++)
        {
            ultrasonic_sensor_func_pair_t *sensor = &ultrasonic_sensor_func_pair[i];

            // Measure distance
            int32_t distance = ultrasonic_measure_distance(sensor->trigger, sensor->echo);

            if (distance < 0)
            {
                consecutive_errors[i]++;
                if (consecutive_errors[i] == 5) // Only log once when hitting 5 errors
                {
                    ESP_LOGW(TAG, "Sensor %d (endpoint %d) has %d consecutive errors, check wiring",
                             i, sensor->endpoint, consecutive_errors[i]);
                }
                else if (consecutive_errors[i] > 10)
                {
                    consecutive_errors[i] = 0; // Reset counter to avoid overflow
                }
                continue;
            }

            // Reset error counter on successful read
            if (consecutive_errors[i] > 0)
            {
                ESP_LOGI(TAG, "Sensor %d recovered after %d errors", i, consecutive_errors[i]);
                consecutive_errors[i] = 0;
            }

            // Determine raw occupancy state using configurable threshold
            esp_zb_zcl_occupancy_sensing_occupancy_t detected_state = (distance <= ultrasonic_detection_threshold_cm) ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;

            ESP_LOGV(TAG, "Sensor %d (endpoint %d): distance=%ld cm, raw_state=%s, reported_state=%s",
                     i, sensor->endpoint, distance,
                     detected_state == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED ? "UNOCCUPIED" : "OCCUPIED",
                     previous_state[i] == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED ? "UNOCCUPIED" : "OCCUPIED");

            // Check if raw state changed
            if (detected_state != raw_state[i])
            {
                raw_state[i] = detected_state;
                state_change_time[i] = current_time_ms;
                ESP_LOGD(TAG, "Sensor %d raw state changed to %s, starting delay timer",
                         i, detected_state == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED ? "UNOCCUPIED" : "OCCUPIED");
            }

            // Determine required delay based on transition direction
            uint16_t required_delay_s = 0;
            if (raw_state[i] != previous_state[i])
            {
                if (raw_state[i] == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED)
                {
                    // Occupied -> Unoccupied transition
                    required_delay_s = ultrasonic_o2u_delay_s;
                }
                else
                {
                    // Unoccupied -> Occupied transition
                    required_delay_s = ultrasonic_u2o_delay_s;
                }

                // Check if delay has elapsed
                int64_t elapsed_ms = current_time_ms - state_change_time[i];
                int64_t required_delay_ms = (int64_t)required_delay_s * 1000;

                if (elapsed_ms >= required_delay_ms)
                {
                    ESP_LOGI(TAG, "Occupancy state changed on endpoint %d: %s -> %s (distance: %ld cm, delay: %d s)",
                             sensor->endpoint,
                             previous_state[i] == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED ? "UNOCCUPIED" : "OCCUPIED",
                             raw_state[i] == ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED ? "UNOCCUPIED" : "OCCUPIED",
                             distance,
                             required_delay_s);

                    previous_state[i] = raw_state[i];

                    // Update func field
                    sensor->func = raw_state[i];

                    // Call callback only if task is ready and we're past initialization
                    if (func_ptr && task_ready)
                    {
                        func_ptr(sensor);
                    }
                }
                else
                {
                    ESP_LOGD(TAG, "Sensor %d waiting for delay: %lld ms / %lld ms",
                             i, elapsed_ms, required_delay_ms);
                }
            }

            // Small delay between sensors to avoid interference
            vTaskDelay(pdMS_TO_TICKS(60));
        }

        // Poll every 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief init GPIO configuration for HC-SR04 ultrasonic sensor
 *
 * @param sensor_func_pair_param  pointer to sensor pair array
 * @param sensor_num              number of sensors
 */
static bool ultrasonic_sensor_gpio_init(ultrasonic_sensor_func_pair_t *sensor_func_pair_param, uint8_t sensor_num)
{
    ultrasonic_sensor_func_pair = sensor_func_pair_param;
    switch_num = sensor_num;

    for (int i = 0; i < sensor_num; i++)
    {
        // Configure trigger pin as output
        gpio_config_t trigger_conf = {
            .pin_bit_mask = (1ULL << sensor_func_pair_param[i].trigger),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&trigger_conf);
        gpio_set_level(sensor_func_pair_param[i].trigger, 0);

        // Configure echo pin as input
        gpio_config_t echo_conf = {
            .pin_bit_mask = (1ULL << sensor_func_pair_param[i].echo),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&echo_conf);

        ESP_LOGI(TAG, "Configured HC-SR04 sensor %d: trigger=GPIO%d, echo=GPIO%d",
                 i, sensor_func_pair_param[i].trigger, sensor_func_pair_param[i].echo);
    }

    // Start polling task
    xTaskCreate(ultrasonic_sensor_task, "ultrasonic_task", 4096, NULL, 10, NULL);

    return true;
}

bool ultrasonic_sensor_init(ultrasonic_sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_ultrasonic_sensor_callback_t cb)
{
    if (!ultrasonic_sensor_gpio_init(sensor_func_pair, sensor_num))
    {
        return false;
    }
    func_ptr = cb;

    // Load settings from NVS if available
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        uint8_t saved_threshold = ULTRASONIC_DETECTION_DISTANCE_CM_DEFAULT;
        err = nvs_get_u8(nvs_handle, NVS_KEY_THRESHOLD, &saved_threshold);
        if (err == ESP_OK)
        {
            ultrasonic_detection_threshold_cm = saved_threshold;
            ESP_LOGI(TAG, "Loaded threshold from NVS: %d cm", ultrasonic_detection_threshold_cm);
        }
        else
        {
            ESP_LOGI(TAG, "No saved threshold in NVS, using default: %d cm", ultrasonic_detection_threshold_cm);
        }

        uint16_t saved_o2u_delay = ULTRASONIC_O2U_DELAY_DEFAULT;
        err = nvs_get_u16(nvs_handle, NVS_KEY_O2U_DELAY, &saved_o2u_delay);
        if (err == ESP_OK)
        {
            ultrasonic_o2u_delay_s = saved_o2u_delay;
            ESP_LOGI(TAG, "Loaded O2U delay from NVS: %d s", ultrasonic_o2u_delay_s);
        }
        else
        {
            ESP_LOGI(TAG, "No saved O2U delay in NVS, using default: %d s", ultrasonic_o2u_delay_s);
        }

        uint16_t saved_u2o_delay = ULTRASONIC_U2O_DELAY_DEFAULT;
        err = nvs_get_u16(nvs_handle, NVS_KEY_U2O_DELAY, &saved_u2o_delay);
        if (err == ESP_OK)
        {
            ultrasonic_u2o_delay_s = saved_u2o_delay;
            ESP_LOGI(TAG, "Loaded U2O delay from NVS: %d s", ultrasonic_u2o_delay_s);
        }
        else
        {
            ESP_LOGI(TAG, "No saved U2O delay in NVS, using default: %d s", ultrasonic_u2o_delay_s);
        }

        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGI(TAG, "NVS not available, using defaults: threshold=%d cm, O2U delay=%d s, U2O delay=%d s",
                 ultrasonic_detection_threshold_cm, ultrasonic_o2u_delay_s, ultrasonic_u2o_delay_s);
    }

    ESP_LOGI(TAG, "Ultrasonic sensor driver initialized with %d HC-SR04 sensors", sensor_num);
    return true;
}

void ultrasonic_sensor_report_initial_states(ultrasonic_sensor_func_pair_t *sensor_func_pair_param, uint8_t sensor_num)
{
    ESP_LOGI(TAG, "Reporting initial sensor states for %d sensors", sensor_num);

    for (int i = 0; i < sensor_num; i++)
    {
        // Measure distance to determine initial state
        int32_t distance = ultrasonic_measure_distance(sensor_func_pair_param[i].trigger,
                                                       sensor_func_pair_param[i].echo);

        uint8_t occupancy = (distance >= 0 && distance < ultrasonic_detection_threshold_cm) ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;

        uint8_t endpoint = sensor_func_pair_param[i].endpoint;

        ESP_LOGI(TAG, "Occupancy sensor endpoint %d initial state: %s (distance: %ld cm)",
                 endpoint,
                 occupancy ? "OCCUPIED" : "UNOCCUPIED",
                 distance);

        // Update attribute
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(endpoint,
                                     ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                                     &occupancy, false);
        esp_zb_lock_release();

        vTaskDelay(pdMS_TO_TICKS(100));

        // Report to coordinator
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {
            .zcl_basic_cmd = {
                .src_endpoint = endpoint,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            .attributeID = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        };
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
        esp_zb_lock_release();

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Initial ultrasonic sensor states reported");
}

void ultrasonic_sensor_set_threshold(uint8_t threshold_cm)
{
    // Validate range (1-254 as per ZCL spec)
    if (threshold_cm < 1)
    {
        threshold_cm = 1;
    }
    else if (threshold_cm > 254)
    {
        threshold_cm = 254;
    }

    ultrasonic_detection_threshold_cm = threshold_cm;
    ESP_LOGI(TAG, "Ultrasonic detection threshold set to %d cm", ultrasonic_detection_threshold_cm);

    // Save to NVS for persistence across reboots
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_u8(nvs_handle, NVS_KEY_THRESHOLD, threshold_cm);
        if (err == ESP_OK)
        {
            nvs_commit(nvs_handle);
            ESP_LOGI(TAG, "Threshold saved to NVS");
        }
        else
        {
            ESP_LOGW(TAG, "Failed to save threshold to NVS: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    }
}

uint8_t ultrasonic_sensor_get_threshold(void)
{
    return ultrasonic_detection_threshold_cm;
}

void ultrasonic_sensor_set_o2u_delay(uint16_t delay_s)
{
    ultrasonic_o2u_delay_s = delay_s;
    ESP_LOGI(TAG, "Ultrasonic O2U delay set to %d s", ultrasonic_o2u_delay_s);

    // Save to NVS for persistence across reboots
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_u16(nvs_handle, NVS_KEY_O2U_DELAY, delay_s);
        if (err == ESP_OK)
        {
            nvs_commit(nvs_handle);
            ESP_LOGI(TAG, "O2U delay saved to NVS");
        }
        else
        {
            ESP_LOGW(TAG, "Failed to save O2U delay to NVS: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    }
}

uint16_t ultrasonic_sensor_get_o2u_delay(void)
{
    return ultrasonic_o2u_delay_s;
}

void ultrasonic_sensor_set_u2o_delay(uint16_t delay_s)
{
    ultrasonic_u2o_delay_s = delay_s;
    ESP_LOGI(TAG, "Ultrasonic U2O delay set to %d s", ultrasonic_u2o_delay_s);

    // Save to NVS for persistence across reboots
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_u16(nvs_handle, NVS_KEY_U2O_DELAY, delay_s);
        if (err == ESP_OK)
        {
            nvs_commit(nvs_handle);
            ESP_LOGI(TAG, "U2O delay saved to NVS");
        }
        else
        {
            ESP_LOGW(TAG, "Failed to save U2O delay to NVS: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    }
}

uint16_t ultrasonic_sensor_get_u2o_delay(void)
{
    return ultrasonic_u2o_delay_s;
}