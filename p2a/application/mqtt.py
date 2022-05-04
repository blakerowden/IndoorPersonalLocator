"""
Prac 2a - Desktop Application
.py file 5/7 - MQTT
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "B.Rowden"

from global_ import *
import tago
import math
from queue import *
import logging


def MQTT_Packer(live_data) -> dict:
    """
    Publish the data to the MQTT broker.
    """
    publish_data = []

    for i in range(len(live_data.node_distance)):

        publish_data.append(
            {
                "variable": "node_distance_" + str(i + 1),
                "unit": "cm",
                "value": math.ceil(live_data.node_distance[i] * 100 / 225),
            }
        )

    for idx, rssi in enumerate(live_data.node_rssi):

        publish_data.append(
            {
                "variable": "rssi_" + str(idx + 1),
                "unit": "dB",
                "value": rssi,
            }
        )

    for idx, ultra in enumerate(live_data.node_ultra):

        publish_data.append(
            {
                "variable": "ultra_" + str(idx + 1),
                "unit": "cm",
                "value": ultra,
            }
        )

    estimated_position = {
        "variable": "position",
        "value": 10,
        "metadata": {
            "x": live_data.k_multilat_pos[0] / 900.0,
            "y": live_data.k_multilat_pos[1] / 900.0,
        },
    }

    publish_data.append(estimated_position)

    delay_time = {
        "variable": "delay",
        "unit": "ms",
        "value": live_data.rssi_delay,
    }

    publish_data.append(delay_time)

    return publish_data


def MQTT_Publisher(pub_q: Queue, stop) -> None:

    my_device = tago.Device(MQTT_DEVICE_TOKEN)
    while True:
        try:
            publish_data = pub_q.get(block=True, timeout=2)
        except Empty:
            if stop():
                logging.info("Stoping MQTT Thread")
                break
            continue

        result = my_device.insert(publish_data)

        if result["status"]:
            logging.info(f"Successfully published with result: {result['result']}")
        else:
            logging.info(f"Fail to publish data with error:  {result['message']}")

        if stop():
            logging.info("Stoping MQTT Thread")
            break
