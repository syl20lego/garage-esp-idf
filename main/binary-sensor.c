#include "binary-sensor.h"
#include "esp_zigbee_core.h"

esp_zb_ep_list_t *esp_zb_binary_sensor_ep_create(uint8_t endpoint, esp_zb_binary_sensor_cfg_t *sensor_cfg)
{
    // Create cluster list
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    // Basic cluster
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&sensor_cfg->basic_cfg);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Binary Input cluster
    esp_zb_attribute_list_t *binary_input_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT);
    esp_zb_binary_input_cluster_add_attr(binary_input_cluster, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &(bool){false});
    esp_zb_binary_input_cluster_add_attr(binary_input_cluster, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, sensor_cfg->sensor_name);
    esp_zb_cluster_list_add_binary_input_cluster(cluster_list, binary_input_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Create endpoint configuration
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0};

    // Create and configure endpoint list
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

    return ep_list;
}