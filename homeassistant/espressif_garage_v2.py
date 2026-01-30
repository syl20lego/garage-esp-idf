"""Quirk v2 for ESPRESSIF ESP32-C6 Garage Controller.

This v2 version includes:
- Entity descriptions for better Home Assistant integration
- Number entities for configurable attributes
- Improved attribute reporting configuration
"""

from typing import Final

from zigpy.profiles import zha
from zigpy.quirks import CustomCluster, CustomDevice
from zigpy.zcl.clusters.general import Basic, Identify, OnOff, BinaryInput
from zigpy.zcl.clusters.measurement import OccupancySensing
from zigpy.zcl.foundation import ZCLAttributeDef
import zigpy.types as t

from zhaquirks import Bus, LocalDataCluster
from zhaquirks.const import (
    DEVICE_TYPE,
    ENDPOINTS,
    INPUT_CLUSTERS,
    MODELS_INFO,
    OUTPUT_CLUSTERS,
    PROFILE_ID,
)

# Attribute IDs
ATTR_ON_TIME: Final = 0x4001
ATTR_OFF_WAIT_TIME: Final = 0x4002
ATTR_ULTRASONIC_O2U_DELAY: Final = 0x0020
ATTR_ULTRASONIC_U2O_DELAY: Final = 0x0021
ATTR_ULTRASONIC_U2O_THRESHOLD: Final = 0x0022


class EspressifOnOffClusterV2(CustomCluster, OnOff):
    """Custom On/Off cluster with OnTime attribute for pulse duration configuration.
    
    Attributes:
        on_time (0x4001): Pulse duration in 1/10th seconds (e.g., 5 = 0.5s)
        off_wait_time (0x4002): Wait time before turning off in 1/10th seconds
    """

    cluster_id = OnOff.cluster_id
    name = "Espressif On/Off"

    attributes = OnOff.attributes.copy()
    attributes.update(
        {
            ATTR_ON_TIME: ZCLAttributeDef(
                id=ATTR_ON_TIME,
                name="on_time",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
            ATTR_OFF_WAIT_TIME: ZCLAttributeDef(
                id=ATTR_OFF_WAIT_TIME,
                name="off_wait_time",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
        }
    )

    async def write_attributes(self, attributes, manufacturer=None):
        """Write attributes and emit event for HA updates."""
        result = await super().write_attributes(attributes, manufacturer)
        
        # Notify listeners about attribute changes
        if ATTR_ON_TIME in attributes:
            self.endpoint.device.on_time_bus.listener_event(
                "on_time_reported", attributes[ATTR_ON_TIME]
            )
        
        return result

    def _update_attribute(self, attrid, value):
        """Handle attribute updates from the device."""
        super()._update_attribute(attrid, value)
        
        if attrid == ATTR_ON_TIME:
            self.endpoint.device.on_time_bus.listener_event("on_time_reported", value)


class EspressifOccupancySensingClusterV2(CustomCluster, OccupancySensing):
    """Custom Occupancy Sensing cluster with ultrasonic configuration attributes.
    
    Attributes:
        ultrasonic_o2u_delay (0x0020): Occupied to Unoccupied delay in seconds
        ultrasonic_u2o_delay (0x0021): Unoccupied to Occupied delay in seconds
        ultrasonic_u2o_threshold (0x0022): Detection threshold in centimeters (1-254)
    """

    cluster_id = OccupancySensing.cluster_id
    name = "Espressif Occupancy Sensing"

    attributes = OccupancySensing.attributes.copy()
    attributes.update(
        {
            ATTR_ULTRASONIC_O2U_DELAY: ZCLAttributeDef(
                id=ATTR_ULTRASONIC_O2U_DELAY,
                name="ultrasonic_o2u_delay",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
            ATTR_ULTRASONIC_U2O_DELAY: ZCLAttributeDef(
                id=ATTR_ULTRASONIC_U2O_DELAY,
                name="ultrasonic_u2o_delay",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
            ATTR_ULTRASONIC_U2O_THRESHOLD: ZCLAttributeDef(
                id=ATTR_ULTRASONIC_U2O_THRESHOLD,
                name="ultrasonic_u2o_threshold",
                type=t.uint8_t,
                access="rw",
                mandatory=False,
            ),
        }
    )

    async def write_attributes(self, attributes, manufacturer=None):
        """Write attributes and emit events for HA updates."""
        result = await super().write_attributes(attributes, manufacturer)
        
        # Determine which bus to use based on endpoint
        if self.endpoint.endpoint_id == 4:
            threshold_bus = self.endpoint.device.threshold_bus_ep4
            o2u_bus = self.endpoint.device.o2u_delay_bus_ep4
            u2o_bus = self.endpoint.device.u2o_delay_bus_ep4
        else:
            threshold_bus = self.endpoint.device.threshold_bus_ep5
            o2u_bus = self.endpoint.device.o2u_delay_bus_ep5
            u2o_bus = self.endpoint.device.u2o_delay_bus_ep5
        
        if ATTR_ULTRASONIC_U2O_THRESHOLD in attributes:
            threshold_bus.listener_event(
                "threshold_reported", attributes[ATTR_ULTRASONIC_U2O_THRESHOLD]
            )
        if ATTR_ULTRASONIC_O2U_DELAY in attributes:
            o2u_bus.listener_event(
                "o2u_delay_reported", attributes[ATTR_ULTRASONIC_O2U_DELAY]
            )
        if ATTR_ULTRASONIC_U2O_DELAY in attributes:
            u2o_bus.listener_event(
                "u2o_delay_reported", attributes[ATTR_ULTRASONIC_U2O_DELAY]
            )
        
        return result

    def _update_attribute(self, attrid, value):
        """Handle attribute updates from the device."""
        super()._update_attribute(attrid, value)
        
        # Determine which bus to use based on endpoint
        if self.endpoint.endpoint_id == 4:
            threshold_bus = self.endpoint.device.threshold_bus_ep4
            o2u_bus = self.endpoint.device.o2u_delay_bus_ep4
            u2o_bus = self.endpoint.device.u2o_delay_bus_ep4
        else:
            threshold_bus = self.endpoint.device.threshold_bus_ep5
            o2u_bus = self.endpoint.device.o2u_delay_bus_ep5
            u2o_bus = self.endpoint.device.u2o_delay_bus_ep5
        
        if attrid == ATTR_ULTRASONIC_U2O_THRESHOLD:
            threshold_bus.listener_event("threshold_reported", value)
        elif attrid == ATTR_ULTRASONIC_O2U_DELAY:
            o2u_bus.listener_event("o2u_delay_reported", value)
        elif attrid == ATTR_ULTRASONIC_U2O_DELAY:
            u2o_bus.listener_event("u2o_delay_reported", value)


class EspressifGarage(CustomDevice):
    """ESPRESSIF ESP32-C6 Garage Controller V2.
    
    Device Endpoints:
        2: Binary Sensor 1 (Garage Door A) - BinaryInput cluster
        3: Binary Sensor 2 (Garage Door B) - BinaryInput cluster
        4: Ultrasonic Occupancy Sensor 1 - OccupancySensing with configurable threshold/delays
        5: Ultrasonic Occupancy Sensor 2 - OccupancySensing with configurable threshold/delays
        10: Relay Output - On/Off with configurable pulse duration (OnTime)
    """

    def __init__(self, *args, **kwargs):
        """Initialize the device with event buses for attribute updates."""
        super().__init__(*args, **kwargs)
        
        # Event buses for relay on_time attribute
        self.on_time_bus = Bus()
        
        # Event buses for occupancy sensor 1 (endpoint 4)
        self.threshold_bus_ep4 = Bus()
        self.o2u_delay_bus_ep4 = Bus()
        self.u2o_delay_bus_ep4 = Bus()
        
        # Event buses for occupancy sensor 2 (endpoint 5)
        self.threshold_bus_ep5 = Bus()
        self.o2u_delay_bus_ep5 = Bus()
        self.u2o_delay_bus_ep5 = Bus()

    signature = {
        MODELS_INFO: [("ESPRESSIF", "esp32c6")],
        ENDPOINTS: {
            # Endpoint 2: Binary Sensor 1 (Garage Door A)
            2: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x000C,  # Simple Sensor
                INPUT_CLUSTERS: [
                    Basic.cluster_id,        # 0x0000
                    BinaryInput.cluster_id,  # 0x000F
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 3: Binary Sensor 2 (Garage Door B)
            3: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x000C,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    BinaryInput.cluster_id,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 4: Ultrasonic Occupancy Sensor 1
            4: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0107,  # Occupancy Sensor
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    OccupancySensing.cluster_id,  # 0x0406
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 5: Ultrasonic Occupancy Sensor 2
            5: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0107,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    OccupancySensing.cluster_id,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 10: Relay (On/Off Output)
            10: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0002,  # On/Off Output
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    Identify.cluster_id,  # 0x0003
                    OnOff.cluster_id,     # 0x0006
                ],
                OUTPUT_CLUSTERS: [],
            },
        },
    }

    replacement = {
        ENDPOINTS: {
            # Endpoint 2: Binary Sensor 1
            2: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x000C,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    BinaryInput.cluster_id,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 3: Binary Sensor 2
            3: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x000C,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    BinaryInput.cluster_id,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 4: Occupancy Sensor 1 with custom cluster
            4: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0107,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    EspressifOccupancySensingClusterV2,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 5: Occupancy Sensor 2 with custom cluster
            5: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0107,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    EspressifOccupancySensingClusterV2,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 10: Relay with custom On/Off cluster
            10: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0002,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    Identify.cluster_id,
                    EspressifOnOffClusterV2,
                ],
                OUTPUT_CLUSTERS: [],
            },
        },
    }
