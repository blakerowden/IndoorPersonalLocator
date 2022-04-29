"""
Prac 2a - Desktop Application
.py file 1/1
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden s4427634"

import time
import serial
import json
import tkinter as tk
from tkinter import messagebox
from threading import Thread
from queue import *
import math
import logging
import tago
import csv
import numpy as np
from datetime import datetime

# MQTT Publishing Code ========================================================
MY_DEVICE_TOKEN = '21d9ce6b-e764-4f0a-83a2-ed2bfdea09f6'

# Defines =====================================================================
START_POS_X = 450
START_POS_Y = 450

SUPER_SHORT_SLEEP = 0.0001
SHORT_SLEEP = 0.001
MEDIUM_SLEEP = 0.1
LONG_SLEEP = 1
SUPER_LONG_SLEEP = 5

# Classes =====================================================================


class TrackingData:
    """
    Class to hold the tracking data.
    """

    def __init__(self):
        self.ultrasonic = [0] * 4
        self.acceleration = [0] * 3
        self.gyro = [0] * 3
        self.magnetometer = [0] * 3
        self.delay = 0
        self.time = 0
        self.node_rssi = [0] * 12
        self.node_distance = [450, 450, 450, 900, 950,
                              1000, 900, 1000, 900, 900, 600, 300]
        self.node_locations = [(0, 0), (300, 0), (600, 0), (900, 0),
                               (900, 300), (900, 600), (900, 900), (600, 900),
                               (300, 900), (0, 900), (0, 600), (0, 300)]
        self.fileList = ["datapoints" + i + ".csv" for i in range(49)]
        self.currentFile = 0;
        self.currentTestpoint = 0;
        self.testxy = [0.5,0.5] #Make sure you do 49 different points
        self.node_transmit_power = [0] * 12
        self.node_alpha = [0] * 12
        self.node_error_constant = [0] * 12
        self.estimated_pos = (0, 0)
        self.kalman_pos = (0, 0)
        self.rssi_error = 0
        self.us_error = 0
        self.current_time = ""

    def write_rssi_csv(self):
        """
        Writes the rssi data to a csv file.
        :param file_name: The name of the file to write to.
        :return: None
        """
        if self.currentTestpoint == 501:
            return
        else:
            self.currentTestpoint += 1

        pos_x = self.currentTestpoint[0];
        pos_y = self.currentTestpoint[1];
        file_name = self.fileList[self.currentFile];

        rowDictionary = {'Pos_X': pos_x, 'Pos_Y': pos_y, 'Node_A': 0, 'Node_B': 0, 'Node_C': 0, 'Node_D': 0,
                     'Node_E': 0, 'Node_F': 0, 'Node_G': 0, 'Node_H': 0, 'Node_I': 0, 'Node_J': 0, 'Node_K': 0, 'Node_L': 0}

        with open(file_name, 'w') as file:
            fieldnames = ['Pos_X', 'Pos_Y', 'Node_A', 'Node_B', 'Node_C', 'Node_D', 'Node_E',
                          'Node_F', 'Node_G', 'Node_H', 'Node_I', 'Node_J', 'Node_K', 'Node_L']

            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()

            for i in range(12):
                rowDictionary[fieldnames[i + 2]] = self.node_rssi[i];

            writer.writerow(rowDictionary)

    def rssi_to_distance(self):
        """
        Converts the received power (RSSI) data from dB to distance in m.
        :return: None
        """
        for i in range(len(self.node_rssi)):
            if self.node_rssi[i] > 0:
                self.node_distance[i] = 225 * 0.4406*math.exp(-0.042*self.node_rssi[i]) - 1
            else:
                self.node_distance[i] = 0

    def populate_data(self, raw_data):
        """
        Populates the tracking data with the raw data.
        :param raw_data: The raw data to populate the tracking data with.
        :return: None
        """
        self.ultrasonic[0] = raw_data["Ultrasonic-1"]
        self.ultrasonic[1] = raw_data["Ultrasonic-2"]
        self.ultrasonic[2] = raw_data["Ultrasonic-3"]
        self.ultrasonic[3] = raw_data["Ultrasonic-4"]
        self.acceleration[0] = raw_data["Accel-X"]
        self.acceleration[1] = raw_data["Accel-Y"]
        self.acceleration[2] = raw_data["Accel-Z"]
        self.gyro[0] = raw_data["Gyro-X"]
        self.gyro[1] = raw_data["Gyro-Y"]
        self.gyro[2] = raw_data["Gyro-Z"]
        self.magnetometer[0] = raw_data["Mag-X"]
        self.magnetometer[1] = raw_data["Mag-Y"]
        self.magnetometer[2] = raw_data["Mag-Z"]
        self.time = raw_data["Time-Stamp"]
        self.delay = raw_data["Delay-Time"]
        self.node_rssi[0] = raw_data["4011-A"]
        self.node_rssi[1] = raw_data["4011-B"]
        self.node_rssi[2] = raw_data["4011-C"]
        self.node_rssi[3] = raw_data["4011-D"]
        self.node_rssi[4] = raw_data["4011-E"]
        self.node_rssi[5] = raw_data["4011-F"]
        self.node_rssi[6] = raw_data["4011-G"]
        self.node_rssi[7] = raw_data["4011-H"]
        self.node_rssi[8] = raw_data["4011-I"]
        self.node_rssi[9] = raw_data["4011-J"]
        self.node_rssi[10] = raw_data["4011-K"]
        self.node_rssi[11] = raw_data["4011-L"]

    def print_data(self):
        """
        Prints the recieved raw data to the console.
        :return: None
        """
        print(f"======== Data Packet Recieved {self.current_time} ========")
        print("Ultrasonic: ", self.ultrasonic)
        print("Accelerometer: ", self.acceleration)
        print("Gyro: ", self.gyro)
        print("Magnetometer: ", self.magnetometer)
        print("Time: ", self.time)
        print("Delay: ", self.delay)
        print("Node RSSI: ", self.node_rssi)
        print("Node Distance: ", self.node_distance)
        print("======================================================\n")

    def estimate_location(self):
        """
        Use the least squares equation to estimate location of the object
        :return tuple: position (x,y)
        """
        x_fixed = np.array([self.node_locations[i][0] for i in range(12)])
        y_fixed = np.array([self.node_locations[i][1] for i in range(12)])
        radius = np.array([self.node_distance[i] for i in range(12)])
        
        if 0 not in radius:
            
            BMat = np.array([(radius[i]**2 - radius[11]**2-x_fixed[i]**2-y_fixed[i]
                            ** 2+x_fixed[11]**2 + y_fixed[11]**2) for i in range(12)])
            AMat = np.array([((2*(x_fixed[11] - x_fixed[i])),
                            (2*(y_fixed[11] - y_fixed[i]))) for i in range(12)])

            FinalProd = np.linalg.lstsq(AMat, BMat, rcond=-1)[0]

            self.estimated_pos = FinalProd.tolist()
            self.estimated_pos[0] = math.ceil(self.estimated_pos[0])
            self.estimated_pos[1] = math.ceil(self.estimated_pos[1])

    def kalman_filter(self):
        """
        Use a Kalman filter to fuse the RF distance and US data to estimate the location of the object.
        :return tuple: position (x,y)
        """


class MainApplication(tk.Frame):
    """
    Main application class.
    """

    def __init__(self, inq_q, master=None):
        super().__init__(master)
        self._master = master
        self.in_q = inq_q
        self._stop = False
        self._master.title("Prac 2 - Position GUI Application")
        self._master.geometry("950x950")
        self._master.configure(bg="white")
        self.pack()

        # Create the grid
        self._grid = Grid(self._master)
        self._grid.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        self._mobile = MobileNode(self._grid)
        self._master.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.update_position()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self._stop = True
            self._master.destroy()

    def update_position(self):
        """
        Update the position of the mobile node.
        """

        while True:
            time.sleep(SHORT_SLEEP)
            try:
                pos = self.in_q.get(block=False)
            except Empty:
                pos = None
            if pos is not None:
                self._mobile.target_x = pos[0]
                self._mobile.target_y = pos[1]
                logging.debug(f"Received new position x:{pos[0]} y:{pos[1]}")
            if self._stop:
                break
            # Update the GUI

            self._mobile.redraw_position()
            self._master.update()


class MobileNode(object):
    """
    Class for the mobile node
    """

    def __init__(self, canvas, master=None):
        self.current_x = START_POS_X
        self.current_y = START_POS_Y
        self.target_x = START_POS_X
        self.target_y = START_POS_Y
        self.canvas = canvas

        self.graphic = self.canvas.create_oval(
            425, 425, 475, 475, fill="#afbecc")
        self.text_position = self.canvas.create_text(
            450, 500, text="(500,500)", fill="black")

    def redraw_position(self):
        """
        Redraw the position of the mobile node.
        """

        if self.current_x < self.target_x:
            self.current_x += 1
            self.canvas.move(self.graphic, 1, 0)
            self.canvas.move(self.text_position, 1, 0)
        elif self.current_x > self.target_x:
            self.current_x -= 1
            self.canvas.move(self.graphic, -1, 0)
            self.canvas.move(self.text_position, -1, 0)
        if self.current_y < self.target_y:
            self.current_y += 1
            self.canvas.move(self.graphic, 0, 1)
            self.canvas.move(self.text_position, 0, 1)
        elif self.current_y > self.target_y:
            self.current_y -= 1
            self.canvas.move(self.graphic, 0, -1)
            self.canvas.move(self.text_position, 0, -1)
        self.canvas.itemconfig(self.text_position,
                               text="({},{})".format(self.current_x, self.current_y))


class Grid(tk.Canvas):
    """
    Class for the grid.
    """

    def __init__(self, master):
        super().__init__(master, bg='white', height=900, width=900)
        self._master = master

        # create a grid
        for i in range(0, 900, 150):
            self.create_line(i, 0, i, 900, fill="black")
        for i in range(0, 900, 150):
            self.create_line(0, i, 900, i, fill="black")

        self.create_static_node_graphic(25, 25, 25)
        self.create_static_node_graphic(600-25, 25, 25)
        self.create_static_node_graphic(300-25, 25, 25)
        self.create_static_node_graphic(900-25, 25, 25)
        self.create_static_node_graphic(900-25, 600-25, 25)
        self.create_static_node_graphic(900-25, 300-25, 25)
        self.create_static_node_graphic(900-25, 900-25, 25)
        self.create_static_node_graphic(600-25, 900-25, 25)
        self.create_static_node_graphic(300-25, 900-25, 25)
        self.create_static_node_graphic(25, 900-25, 25)
        self.create_static_node_graphic(25, 600-25, 25)
        self.create_static_node_graphic(25, 300-25, 25)

    def create_static_node_graphic(self, pos_x, pos_y, size):
        """
        Create the graphic for the static node.
        """
        self.create_polygon(
            pos_x + size, pos_y,  # Vertex A
            pos_x + (size/2), pos_y + math.sqrt(3)*size/2,  # Vertex B
            pos_x - (size/2), pos_y + math.sqrt(3)*size/2,  # Vertex B
            pos_x - size, pos_y,  # Vertex D
            pos_x - (size/2), pos_y - math.sqrt(3)*size/2,  # Vertex E
            pos_x + (size/2), pos_y - math.sqrt(3)*size/2,  # Vertex F

            fill="black")

# Serial Interface ============================================================


def serial_interface(out_q, stop):
    """
    Thread for the serial interfacing.
    """
    if not stop():
        try:
            ser = serial.Serial(
                port='/dev/ttyACM0',
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            ser.is_open = True
            logging.info(f"Connected to Serial Port {ser.name}")
            time.sleep(SHORT_SLEEP)
        except:
            logging.warning("Could not connect to serial port")
            time.sleep(LONG_SLEEP)
            logging.info("Attempting to reconnect to serial port")
            serial_interface(out_q, stop)

    if 'ser' in locals():
        while(ser.is_open):
            try:
                line = ser.readline().decode('utf-8').strip()
            except:
                ser.close()
                logging.warning("Could not read line from serial port")
                time.sleep(SUPER_LONG_SLEEP)
                logging.info("Attempting to reconnect to serial port")
                serial_interface(out_q, stop)
            try:
                data = json.loads(line)
                out_q.put(data)
            except:
                logging.debug(f"Could not parse line from serial port: {line}")
            if stop():
                break

        ser.close()


# Data Processing =============================================================


def data_processing(in_q, out_q, pub_q, stop):
    """
    Process the raw JSON data.
    """
    live_data = TrackingData()
    while True:

        # Get the next message from the queue
        data_raw = in_q.get(block=True)
        now = datetime.now()  # Timestamp incomming data
        live_data.current_time = now.strftime("%H:%M:%S.%f")
        live_data.populate_data(data_raw)
        live_data.rssi_to_distance()
        live_data.estimate_location()
        live_data.print_data()

        # Send the estimated position to the GUI
        out_q.put(live_data.estimated_pos)
        pub_data = MQTT_Packer(live_data)
        pub_q.queue.clear()
        pub_q.put(pub_data)
        if stop():
            logging.info("Stoping Data Thread")
            break


def MQTT_Packer(live_data):
    """
    Publish the data to the MQTT broker.
    """
    publish_data = []

    for i in range(len(live_data.node_distance)):

        publish_data.append(
            {
                'variable': 'node_distance_' + str(i+1),
                'unit': 'cm',
                'value': math.ceil(live_data.node_distance[i]*100/225),
            }
        )

    estimated_position = {
        "variable": "position",
        "value": 10,
        "metadata": {"x": live_data.estimated_pos[0]/900.0, "y": live_data.estimated_pos[1]/900.0},
    }

    publish_data.append(estimated_position)

    delay_time = {
        "variable": "delay",
        "value": live_data.delay,
    }

    publish_data.append(delay_time)

    return publish_data


def MQTT_Publisher(pub_q, stop):

    my_device = tago.Device(MY_DEVICE_TOKEN)
    while(True):
        publish_data = pub_q.get(block=True)
        result = my_device.insert(publish_data)

        if result['status']:
            logging.info(
                f"Successfully published with result: {result['result']}")
        else:
            logging.info(
                f"Fail to publish data with error:  {result['message']}")


# GUI Interface ===============================================================

def gui_interface(in_q):
    """
    GUI interface for the application.
    """
    root = tk.Tk()
    app = MainApplication(in_q, master=root)
    app.mainloop()


# Insertion Point =============================================================

def main():
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

    j_data = Queue()    # Queue for Raw JSON data
    k_data = Queue()    # Queue for (k)lean data
    m_data = Queue()    # Queue for MQTT data

    if comms_active:
        # Create thread to read from the serial port
        logging.debug("Starting Serial Thread")
        thread_serial = Thread(target=serial_interface,
                               args=(j_data, lambda: stop_flag))
        thread_serial.start()

    if data_active:
        # Create thread to process the data
        logging.debug("Starting Data Thread")
        thread_data = Thread(target=data_processing, args=(
            j_data, k_data, m_data, lambda: stop_flag))
        thread_data.start()

    if gui_active:
        # Create thread to run the GUI
        logging.debug("Starting GUI Thread")
        thread_gui = Thread(target=gui_interface, args=(k_data,))
        thread_gui.start()

    if mqtt_active:
        # Create thread to run the MQTT interface
        logging.debug("Starting MQTT Thread")
        thread_mqtt = Thread(target=MQTT_Publisher,
                             args=(m_data, lambda: stop_flag))
        thread_mqtt.start()

    while not stop_flag:
        if (gui_active and not thread_gui.is_alive()) or (data_active and not thread_data.is_alive()) or (comms_active and not thread_serial.is_alive()):
            logging.warning(
                "One of the threads has stopped. Stopping the program...")
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
