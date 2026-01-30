"""Quirk for ESPRESSIF ESP32-C6 Garage Controller."""

from zigpy.profiles import zha
from zigpy.quirks import CustomCluster, CustomDevice
from zigpy.zcl.clusters.general import Basic, Identify, OnOff, BinaryInput
from zigpy.zcl.clusters.measurement import OccupancySensing
from zigpy.zcl.foundation import ZCLAttributeDef, ZCLCommandDef
from zigpy.zcl import ClusterType
import zigpy.types as t

from zhaquirks import LocalDataCluster
from zhaquirks.const import (
    DEVICE_TYPE,
    ENDPOINTS,
    INPUT_CLUSTERS,
    MODELS_INFO,
    OUTPUT_CLUSTERS,
    PROFILE_ID,
)


class EspressifOnOffCluster(CustomCluster, OnOff):
    """Custom On/Off cluster with OnTime attribute for pulse duration configuration."""

    # OnTime attribute (0x4001) - pulse duration in 1/10th seconds
    # This allows configuring how long the relay stays on when triggered
    attributes = OnOff.attributes.copy()
    attributes.update(
        {
            0x4001: ZCLAttributeDef(
                id=0x4001,
                name="on_time",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
            0x4002: ZCLAttributeDef(
                id=0x4002,
                name="off_wait_time",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
        }
    )


class EspressifOccupancySensingCluster(CustomCluster, OccupancySensing):
    """Custom Occupancy Sensing cluster with ultrasonic configuration attributes."""

    # Ultrasonic configuration attributes per ZCL spec section 4.8.2.2.3
    attributes = OccupancySensing.attributes.copy()
    attributes.update(
        {
            # UltrasonicOccupiedToUnoccupiedDelay (0x0020) - delay in seconds
            0x0020: ZCLAttributeDef(
                id=0x0020,
                name="ultrasonic_o2u_delay",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
            # UltrasonicUnoccupiedToOccupiedDelay (0x0021) - delay in seconds
            0x0021: ZCLAttributeDef(
                id=0x0021,
                name="ultrasonic_u2o_delay",
                type=t.uint16_t,
                access="rw",
                mandatory=False,
            ),
            # UltrasonicUnoccupiedToOccupiedThreshold (0x0022) - detection threshold in cm
            0x0022: ZCLAttributeDef(
                id=0x0022,
                name="ultrasonic_u2o_threshold",
                type=t.uint8_t,
                access="rw",
                mandatory=False,
            ),
        }
    )


class EspressifGarage(CustomDevice):
    """ESPRESSIF ESP32-C6 Garage Controller."""

    signature = {
        MODELS_INFO: [("ESPRESSIF", "esp32c6")],
        ENDPOINTS: {
            # Endpoint 2: Binary Sensor 1 (Garage Door A)
            2: {
                PROFILE_ID: zha.PROFILE_ID,  # 0x0104
                DEVICE_TYPE: 0x000C,  # Simple Sensor
                INPUT_CLUSTERS: [
                    Basic.cluster_id,       # 0x0000
                    BinaryInput.cluster_id, # 0x000F
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 3: Binary Sensor 2 (Garage Door B)
            3: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x000C,  # Simple Sensor
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
                DEVICE_TYPE: 0x0107,  # Occupancy Sensor
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
                    EspressifOccupancySensingCluster,
                ],
                OUTPUT_CLUSTERS: [],
            },
            # Endpoint 5: Occupancy Sensor 2 with custom cluster
            5: {
                PROFILE_ID: zha.PROFILE_ID,
                DEVICE_TYPE: 0x0107,
                INPUT_CLUSTERS: [
                    Basic.cluster_id,
                    EspressifOccupancySensingCluster,
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
                    EspressifOnOffCluster,
                ],
                OUTPUT_CLUSTERS: [],
            },
        },
    }
