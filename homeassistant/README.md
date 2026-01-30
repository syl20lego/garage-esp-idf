# Home Assistant ZHA Quirk for ESPRESSIF ESP32-C6 Garage Controller

This quirk enables proper handling of the custom attributes on your ESP32-C6 garage device.

## Quirk Versions

- **espressif_garage.py** - Basic quirk with custom attribute definitions
- **espressif_garage_v2.py** - Enhanced version with event buses for better HA integration

## Features

- **Binary Sensors (Endpoints 2, 3)**: Garage door open/close sensors
- **Occupancy Sensors (Endpoints 4, 5)**: Ultrasonic presence detection with configurable:
  - Detection threshold (cm)
  - Occupied-to-Unoccupied delay (seconds)
  - Unoccupied-to-Occupied delay (seconds)
- **Relay (Endpoint 10)**: On/Off output with configurable pulse duration (OnTime attribute)

## Installation

### Method 1: Custom Quirks Directory (Recommended)

1. Create the custom quirks directory in your Home Assistant config:
   ```bash
   mkdir -p /config/custom_zha_quirks
   ```

2. Copy both `__init__.py` and `espressif_garage.py` to the custom quirks directory:
   ```bash
   cp __init__.py /config/custom_zha_quirks/
   cp espressif_garage.py /config/custom_zha_quirks/
   ```

3. Add the following to your `configuration.yaml`:
   ```yaml
   zha:
     custom_quirks_path: /config/custom_zha_quirks
   ```

4. Restart Home Assistant

5. Remove and re-pair the device (or reconfigure the device from the ZHA integration)

### Method 2: Using zhaquirks package

If you're contributing to the zhaquirks project, place the file in:
```
zhaquirks/espressif/garage.py
```

And add to `zhaquirks/espressif/__init__.py`:
```python
from zhaquirks.espressif.garage import EspressifGarage
```

## Accessing Custom Attributes

### Via Developer Tools > Services

You can read/write attributes using the `zha.set_zigbee_cluster_attribute` and `zha.issue_zigbee_cluster_command` services.

#### Reading Occupancy Sensor Threshold (Endpoint 4 or 5)
```yaml
service: zha.issue_zigbee_cluster_command
data:
  ieee: "YOUR_DEVICE_IEEE_ADDRESS"
  endpoint_id: 4
  cluster_id: 1030  # 0x0406 Occupancy Sensing
  cluster_type: in
  command: 0  # Read attributes
  command_type: general
  args:
    - 34  # 0x0022 - ultrasonic_u2o_threshold
```

#### Setting Occupancy Sensor Threshold (e.g., 100cm)
```yaml
service: zha.set_zigbee_cluster_attribute
data:
  ieee: "YOUR_DEVICE_IEEE_ADDRESS"
  endpoint_id: 4
  cluster_id: 1030  # 0x0406
  cluster_type: in
  attribute: 34  # 0x0022 - ultrasonic_u2o_threshold
  value: 100
```

#### Setting Occupancy Sensor O2U Delay (e.g., 10 seconds)
```yaml
service: zha.set_zigbee_cluster_attribute
data:
  ieee: "YOUR_DEVICE_IEEE_ADDRESS"
  endpoint_id: 4
  cluster_id: 1030
  cluster_type: in
  attribute: 32  # 0x0020 - ultrasonic_o2u_delay
  value: 10
```

#### Setting Occupancy Sensor U2O Delay (e.g., 5 seconds)
```yaml
service: zha.set_zigbee_cluster_attribute
data:
  ieee: "YOUR_DEVICE_IEEE_ADDRESS"
  endpoint_id: 4
  cluster_id: 1030
  cluster_type: in
  attribute: 33  # 0x0021 - ultrasonic_u2o_delay
  value: 5
```

#### Setting Relay Pulse Duration (OnTime, e.g., 5 = 0.5 seconds)
```yaml
service: zha.set_zigbee_cluster_attribute
data:
  ieee: "YOUR_DEVICE_IEEE_ADDRESS"
  endpoint_id: 10
  cluster_id: 6  # 0x0006 On/Off
  cluster_type: in
  attribute: 16385  # 0x4001 - on_time (in 1/10th seconds)
  value: 5  # 0.5 seconds
```

### Via ZHA Cluster Management UI

1. Go to **Settings** > **Devices & Services** > **ZHA**
2. Click on your device
3. Click **Manage Zigbee Device**
4. Select the endpoint and cluster
5. Read or write attributes from the UI

## Attribute Reference

### Occupancy Sensing Cluster (0x0406)

| Attribute ID | Name | Type | Description |
|-------------|------|------|-------------|
| 0x0000 | occupancy | map8 | Current occupancy state (0=unoccupied, 1=occupied) |
| 0x0001 | occupancy_sensor_type | enum8 | Sensor type (1=Ultrasonic) |
| 0x0020 | ultrasonic_o2u_delay | uint16 | Occupied to Unoccupied delay in seconds |
| 0x0021 | ultrasonic_u2o_delay | uint16 | Unoccupied to Occupied delay in seconds |
| 0x0022 | ultrasonic_u2o_threshold | uint8 | Detection threshold in cm (1-254) |

### On/Off Cluster (0x0006)

| Attribute ID | Name | Type | Description |
|-------------|------|------|-------------|
| 0x0000 | on_off | bool | Current on/off state |
| 0x4001 | on_time | uint16 | Pulse duration in 1/10th seconds |

## Troubleshooting

1. **Device not using quirk**: Check that the manufacturer/model matches exactly ("ESPRESSIF", "esp32c6")
2. **Attributes not visible**: Try reconfiguring the device from ZHA integration
3. **Cannot write attributes**: Ensure the device is awake (for battery devices) and the attribute has write access
