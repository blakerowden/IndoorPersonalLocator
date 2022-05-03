"""
Prac 2a - Desktop Application
.py file 2/6
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden, Boston O'Neill and Liana can Teijlingen"

import logging
import serial
from queue import *
import json
from global_ import *
import time


def serial_interface_thread(out_q, stop):
    """
    Thread for the serial interfacing.
    """
    if not stop():
        try:
            ser = serial.Serial(
                port="/dev/ttyACM0",
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1,
            )
            ser.is_open = True
            logging.info(f"Connected to Serial Port {ser.name}")
            time.sleep(SHORT_SLEEP)
        except:
            logging.warning("Could not connect to serial port")
            time.sleep(LONG_SLEEP)
            logging.info("Attempting to reconnect to serial port")
            serial_interface_thread(out_q, stop)

    if "ser" in locals():
        while ser.is_open:
            try:
                line = ser.readline().decode("utf-8").strip()
            except:
                ser.close()
                logging.warning("Could not read line from serial port")
                time.sleep(SUPER_LONG_SLEEP)
                logging.info("Attempting to reconnect to serial port")
                serial_interface_thread(out_q, stop)
            try:
                data = json.loads(line)
                out_q.put(data)
            except:
                logging.debug(f"Could not parse line from serial port: {line}")
            if stop():
                ser.close()
                return
