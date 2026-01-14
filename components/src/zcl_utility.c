/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee light driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_check.h"
#include "stdio.h"
#include "string.h"
#include "zcl_utility.h"
#include <stdint.h>

static const char *TAG = "ZCL_UTILITY";

esp_err_t esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, zcl_basic_manufacturer_info_t *info)
{
    esp_err_t ret = ESP_OK;
    esp_zb_cluster_list_t *cluster_list = NULL;
    esp_zb_attribute_list_t *basic_cluster = NULL;

    /**
    3.2 Basic
        This cluster supports an interface to the node or physical device. It provides attributes and commands for
        determining basic information, setting user information such as location, and resetting to factory defaults.
        Note: Where a node supports multiple endpoints, it will often be the case that many of these settings will
        apply to the whole node, that is, they are the same for every endpoint on the node. In such cases, they can be
        implemented once for the node, and mapped to each endpoint.

    3.2.2.2 Attributes
        The Basic cluster attributes are summarized in Table 3-7

        Id      Name                        Type    Range           Acc     Default         M/O
        0x0000  ZCLVersion                  uint8   0x00 to 0xff    R       8               M
        0x0001  ApplicationVersion          uint8   0x00 to 0xff    R       0               O
        0x0002  StackVersion                uint8   0x00 to 0xff    R       0               O
        0x0003  HWVersion                   uint8   0x00 to 0xff    R       0               O
        0x0004  ManufacturerName            string  0 to 32 bytes   R       empty string    O
        0x0005  ModelIdentifier             string  0 to 32 bytes   R       empty string    O
        0x0006  DateCode                    string  0 to 16 bytes   R       empty string    O
        0x0007  PowerSource                 enum8   0x00 to 0xff    R       0x00            M
        0x0008  GenericDevice-Class         enum8   0x00 to 0xff    R       0xff            O
        0x0009  GenericDevice-Type          enum8   0x00 to 0xff    R       0xff            O
        0x000a  ProductCode                 octstr                  R       empty string    O
        0x000b  ProductURL                  string                  R       empty string    O
        0x000c  ManufacturerVersionDetails  string                  R       empty string    O
        0x000d  SerialNumber                string                  R       empty string    O
        0x000e  ProductLabel                string                  R       empty string    O
        0x0010  LocationDescription         string  0 to 16 bytes   RW      empty string    O
        0x0011  PhysicalEnvironment         enum8   desc            RW      0               O
        0x0012  DeviceEnabled               bool    0 or 1          RW      1               O
        0x0013  AlarmMask                   map8    000000xx        RW      0               O
        0x0014  DisableLocalConfig          map8    000000xx        RW      0               O
        0x4000  SWBuildID                   string  0 to 16 bytes   R       empty string    O
     */
    cluster_list = esp_zb_ep_list_get_ep(ep_list, endpoint_id);
    ESP_RETURN_ON_FALSE(cluster_list, ESP_ERR_INVALID_ARG, TAG, "Failed to find endpoint id: %d in list: %p", endpoint_id, ep_list);
    basic_cluster = esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_RETURN_ON_FALSE(basic_cluster, ESP_ERR_INVALID_ARG, TAG, "Failed to find basic cluster in endpoint: %d", endpoint_id);
    ESP_RETURN_ON_FALSE((info && info->manufacturer_name), ESP_ERR_INVALID_ARG, TAG, "Invalid manufacturer name");
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, info->manufacturer_name));
    ESP_RETURN_ON_FALSE((info && info->model_identifier), ESP_ERR_INVALID_ARG, TAG, "Invalid model identifier");
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, info->model_identifier));
    // Set power source attribute
    if (info->power_source != 0)
    {
        ESP_ERROR_CHECK(esp_zb_cluster_update_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &info->power_source));
    }
    return ret;
}
