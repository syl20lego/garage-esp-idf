#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "binary-sensor.h"

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;
/* button function pair, should be defined in switch example source file */
static sensor_func_pair_t *sensor_func_pair;
/* call back function pointer */
static esp_sensor_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
static const char *TAG = "ESP_ZB_SENSOR";

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    xQueueSendFromISR(gpio_evt_queue, (sensor_func_pair_t *)arg, NULL);
}

/**
 * @brief Enable GPIO (switches refer to) isr
 *
 * @param enabled      enable isr if true.
 */
static void binary_sensor_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i)
    {
        if (enabled)
        {
            gpio_intr_enable((sensor_func_pair + i)->pin);
        }
        else
        {
            gpio_intr_disable((sensor_func_pair + i)->pin);
        }
    }
}

/**
    3.14.5 Binary Input (Basic)
    The Binary Input (Basic) cluster provides an interface for reading the value of a binary measurement and
    accessing various characteristics of that measurement. The cluster is typically used to implement a sensor that
    measures a two-state physical quantity.

    @param[in] sensor_cfg Configuration for binary sensor
    @return Endpoint list with binary sensor configuration
*/
esp_zb_cluster_list_t *garage_binary_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_binary_sensor_cfg_t *sensor_cfg)
{
    // Create cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&sensor_cfg->basic_cfg);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*
    https://zigbeealliance.org/wp-content/uploads/2019/12/07-5123-06-zigbee-cluster-library-specification.pdf

    Binary Input cluster
    3.14.5.3 Cluster Identifiers

    Identifier Name
    0x000f Binary Input

    3.14.5.4.2 Attributes
    Attribute ID   Name              Type      Range                Access  Default         M/O
    0x0004       ActiveText          string      -                   R*W     Null string     O
    0x001C       Description         string      -                   R*W     Null string     O
    0x002E       InactiveText        string      -                   R*W     Null string     O
    0x0051       OutOfService        bool      False (0) or True (1) R*W     False (0)       M
    0x0054       Polarity            enum8       -                   R       0               O
    0x0055       PresentValue        bool        -                   R*W     -               M
    0x0067       Reliability         enum8       -                   R*W     0x00            O
    0x006F       StatusFlags         map8        0x00-0x0f           R       0               M
    0x0100       ApplicationType     uint32      0-0xffffffff        R       -               O

    */
    esp_zb_attribute_list_t *binary_input_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT);
    esp_zb_binary_input_cluster_add_attr(binary_input_cluster, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &(bool){false});
    esp_zb_binary_input_cluster_add_attr(binary_input_cluster, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, sensor_cfg->sensor_name);
    esp_zb_cluster_list_add_binary_input_cluster(cluster_list, binary_input_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

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
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0};

    // Add binary enpoint to the endpoint list
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    return cluster_list;
}

/**
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */
static void binary_sensor_detected(void *arg)
{
    gpio_num_t io_num = GPIO_NUM_NC;
    sensor_func_pair_t sensor_func_pair;
    static binary_sensor_state_t sensor_state = SENSOR_IDLE;
    bool evt_flag = false;

    for (;;)
    {
        /* check if there is any queue received, if yes read out the button_func_pair */
        if (xQueueReceive(gpio_evt_queue, &sensor_func_pair, portMAX_DELAY))
        {
            io_num = sensor_func_pair.pin;
            binary_sensor_gpios_intr_enabled(false);
            evt_flag = true;
        }
        /* Debounce loop */
        while (evt_flag)
        {
            bool value = gpio_get_level(io_num);
            bool normal_level = sensor_func_pair.normal_level;

            // ESP_LOGI(TAG, "GPIO level read: %d (ON=%d)", value, normal_level);

            // Determine new state based on GPIO level

            binary_sensor_state_t new_state = (value == normal_level) ? SENSOR_DETECTED : SENSOR_IDLE;

            // ESP_LOGI(TAG, "Sensor state transition: %s -> %s",
            //          sensor_state == SENSOR_IDLE ? "IDLE" : "DETECTED",
            //          new_state == SENSOR_IDLE ? "IDLE" : "DETECTED");

            // Only process if state actually changed
            if (new_state != sensor_state)
            {
                ESP_LOGD(TAG, "Sensor state changed  %s", new_state == SENSOR_IDLE ? "IDLE" : "DETECTED");
                sensor_state = new_state;

                // Update the sensor_func_pair func field to match the state
                sensor_func_pair.func = (new_state == SENSOR_DETECTED) ? SENSOR_TOGGLE_CONTROLL_ON : SENSOR_TOGGLE_CONTROLL_OFF;
                // Call the callback to notify the application
                (*func_ptr)(&sensor_func_pair);
            }
            else
            {
                ESP_LOGD(TAG, "Sensor state unchanged, ignoring");
            }

            // Small delay for debouncing
            vTaskDelay(100 / portTICK_PERIOD_MS);
            binary_sensor_gpios_intr_enabled(true);
            evt_flag = false;
            break;
        }
    }
}

/**
 * @brief init GPIO configuration as well as isr
 *
 * @param button_func_pair      pointer of the button pair.
 * @param button_num            number of button pair.
 */
static bool binary_sensor_gpio_init(sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num)
{
    gpio_config_t io_conf = {};
    sensor_func_pair = sensor_func_pair;
    sensor_num = sensor_num;
    uint64_t pin_bit_mask = 0;

    /* set up button func pair pin mask */
    for (int i = 0; i < sensor_num; ++i)
    {
        pin_bit_mask |= (1ULL << (sensor_func_pair + i)->pin);
    }
    /* interrupt of falling edge */
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);
    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(sensor_func_pair_t));
    if (gpio_evt_queue == 0)
    {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(binary_sensor_detected, "sensor_detected", 4096, NULL, 10, NULL);
    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < sensor_num; ++i)
    {
        gpio_isr_handler_add((sensor_func_pair + i)->pin, gpio_isr_handler, (void *)(sensor_func_pair + i));
    }
    return true;
}

bool binary_sensor_init(sensor_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_sensor_callback_t cb)
{
    if (!binary_sensor_gpio_init(sensor_func_pair, sensor_num))
    {
        return false;
    }
    func_ptr = cb;

    ESP_LOGI(TAG, "Binary sensor driver initialized with %d sensors", sensor_num);
    return true;
}

void binary_sensor_report_initial_states(sensor_func_pair_t *button_func_pair, uint8_t sensor_num)
{
    ESP_LOGI(TAG, "Reporting initial sensor states for %d sensors", sensor_num);

    for (int i = 0; i < sensor_num; i += 2) // Step by 2 since each sensor has ON/OFF pair
    {
        gpio_num_t pin = (button_func_pair + i)->pin;
        bool normal_level = (button_func_pair + i)->normal_level;
        bool gpio_level = gpio_get_level(pin);

        // Determine initial state based on GPIO level
        bool sensor_state = (gpio_level == normal_level);
        uint8_t endpoint = (button_func_pair + i)->endpoint;

        ESP_LOGI(TAG, "Sensor endpoint %d, pin %d initial state: %s (GPIO level: %d)",
                 endpoint, pin,
                 sensor_state ? "DETECTED" : "IDLE",
                 gpio_level);

        // Update the attribute directly
        esp_zb_lock_acquire(portMAX_DELAY);

        esp_zb_zcl_set_attribute_val(endpoint,
                                     ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
                                     &sensor_state, false);

        esp_zb_lock_release();

        // Small delay between sensors
        vTaskDelay(pdMS_TO_TICKS(100));

        // Report to coordinator
        esp_zb_lock_acquire(portMAX_DELAY);

        esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {
            .zcl_basic_cmd = {
                .src_endpoint = endpoint,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
            .clusterID = ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
            .attributeID = ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        };
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);

        esp_zb_lock_release();

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Initial sensor states reported");
}