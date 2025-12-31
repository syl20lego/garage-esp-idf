#include "esp_zigbee_core.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "occupency_sensor.h"

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;
/* button function pair, should be defined in switch example source file */
static occupency_func_pair_t *occupency_func_pair;
/* call back function pointer */
static esp_occupency_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
static const char *TAG = "ESP_ZB_OCCUPENCY";

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    xQueueSendFromISR(gpio_evt_queue, (occupency_func_pair_t *)arg, NULL);
}

/**
 * @brief Enable GPIO (switches refer to) isr
 *
 * @param enabled      enable isr if true.
 */
static void occupency_sensor_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i)
    {
        if (enabled)
        {
            gpio_intr_enable((occupency_func_pair + i)->pin);
        }
        else
        {
            gpio_intr_disable((occupency_func_pair + i)->pin);
        }
    }
}

/**
    4.8 Occupancy Sensing
    Please see Chapter 2 for a general cluster overview defining cluster architecture, revision, classification,
    identification, etc.
    The server cluster provides an interface to occupancy sensing functionality, including configuration and
    provision of notifications of occupancy status.

    @param[in] sensor_cfg Configuration for occupency sensor
    @return Endpoint list with occupency sensor configuration
*/
esp_zb_cluster_list_t *garage_occupency_sensor_ep_create(esp_zb_ep_list_t *ep_list, esp_zb_occupency_sensor_cfg_t *sensor_cfg)
{
    // Create cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&sensor_cfg->basic_cfg);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /*
    https://zigbeealliance.org/wp-content/uploads/2019/12/07-5123-06-zigbee-cluster-library-specification.pdf

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
    // Occupancy Sensor Type attribute (enum8, 0x0001) - needs to be a pointer
    uint8_t sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC;
    esp_zb_occupancy_sensing_cluster_add_attr(input_cluster, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &sensor_type);

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
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */
static void occupency_sensor_detected(void *arg)
{
    gpio_num_t io_num = GPIO_NUM_NC;
    occupency_func_pair_t occupency_func_pair; // Local variable to receive from queue
    static occupency_sensor_state_t occupency_state = OCCUPENCY_IDLE;

    for (;;)
    {
        /* check if there is any queue received, if yes read out the occupency_func_pair */
        if (xQueueReceive(gpio_evt_queue, &occupency_func_pair, portMAX_DELAY))
        {
            io_num = occupency_func_pair.pin;
            occupency_sensor_gpios_intr_enabled(false);

            // Read GPIO level
            bool value = gpio_get_level(io_num);

            // Determine state based on GPIO level and normal level
            occupency_state = (value == occupency_func_pair.normal_level) ? OCCUPENCY_DETECTED : OCCUPENCY_IDLE;

            ESP_LOGI(TAG, "Occupancy sensor GPIO %d: level=%d, normal=%d, state=%s",
                     io_num, value, occupency_func_pair.normal_level,
                     occupency_state == OCCUPENCY_IDLE ? "IDLE" : "DETECTED");

            // Update the func field to match the state
            occupency_func_pair.func = (occupency_state == OCCUPENCY_DETECTED) ? OCCUPENCY_TOGGLE_CONTROLL_ON : OCCUPENCY_TOGGLE_CONTROLL_OFF;

            // Call the callback to notify the application
            if (func_ptr)
            {
                func_ptr(&occupency_func_pair); // Pass address of LOCAL variable, not global pointer
            }

            // Re-enable interrupts
            occupency_sensor_gpios_intr_enabled(true);
        }
    }
}

/**
 * @brief init GPIO configuration as well as isr
 *
 * @param button_func_pair      pointer of the button pair.
 * @param button_num            number of button pair.
 */
static bool occupency_sensor_gpio_init(occupency_func_pair_t *sensor_func_pair, uint8_t sensor_num)
{
    gpio_config_t io_conf = {};
    occupency_func_pair = occupency_func_pair;
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
    gpio_evt_queue = xQueueCreate(10, sizeof(occupency_func_pair_t));
    if (gpio_evt_queue == 0)
    {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(occupency_sensor_detected, "sensor_detected", 4096, NULL, 10, NULL);
    /* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < sensor_num; ++i)
    {
        gpio_isr_handler_add((sensor_func_pair + i)->pin, gpio_isr_handler, (void *)(sensor_func_pair + i));
    }
    return true;
}

bool occupency_sensor_init(occupency_func_pair_t *sensor_func_pair, uint8_t sensor_num, esp_occupency_callback_t cb)
{
    if (!occupency_sensor_gpio_init(sensor_func_pair, sensor_num))
    {
        return false;
    }
    func_ptr = cb;
    // Read and report initial state for each sensor
    ESP_LOGI(TAG, "Reading initial sensor states for %d sensors", sensor_num);
    return true;
}