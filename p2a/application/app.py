"""
Prac 2a - Desktop Application
.py file 1/7 - Main File
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "B.Rowden, B.O'Neill and L.Van Teijlingen"

import time
from threading import Thread
from queue import *
import logging
from comm import *
from data import *
from mqtt import *
from gui import *
from global_ import *


def main() -> None:
    """
    Main function for the application.
    """
    # Set logging level
    logging.basicConfig(level=logging.INFO)
    # Create a stop flag
    stop_flag = False
    comms_active = True
    data_active = True
    mqtt_active = True
    gui_active = True

    thread_serial = None
    thread_data = None
    thread_gui = None
    thread_mqtt = None

    j_data = Queue()  # Queue for Raw JSON data
    k_data = Queue()  # Queue for (k)lean data
    m_data = Queue()  # Queue for MQTT data

    if comms_active:
        # Create thread to read from the serial port
        logging.debug("Starting Serial Thread")
        thread_serial = Thread(
            target=serial_interface_thread, args=(j_data, lambda: stop_flag)
        )
        thread_serial.start()

    if data_active:
        # Create thread to process the data
        logging.debug("Starting Data Thread")
        thread_data = Thread(
            target=data_processing_thread,
            args=(j_data, k_data, m_data, lambda: stop_flag),
        )
        thread_data.start()

    if gui_active:
        # Create thread to run the GUI
        logging.debug("Starting GUI Thread")
        thread_gui = Thread(target=gui_thread, args=(k_data,))
        thread_gui.start()

    if mqtt_active:
        # Create thread to run the MQTT interface
        logging.debug("Starting MQTT Thread")
        thread_mqtt = Thread(target=MQTT_Publisher, args=(m_data, lambda: stop_flag))
        thread_mqtt.start()

    while not stop_flag:
        if (
            (gui_active and not thread_gui.is_alive())
            or (data_active and not thread_data.is_alive())
            or (comms_active and not thread_serial.is_alive())
        ):
            logging.warning("One of the threads has stopped. Stopping the program...")
            stop_flag = True

        time.sleep(SHORT_SLEEP)

    if gui_active:
        thread_gui.join()
        logging.info("GUI Thread Closed")

    if data_active:
        thread_data.join()
        logging.info("Data Thread Closed")
    if comms_active:
        thread_serial.join()
        logging.info("Serial Thread Closed")

    logging.info("program complete")


if __name__ == "__main__":
    """
    Only run the main function if this file is run directly.
    """
    main()
