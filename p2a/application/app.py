"""
Prac 2a - Desktop Application
.py file 1/1
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden s4427634"

import time
from turtle import color
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
import pylab
from datetime import datetime

# MQTT Publishing Code ========================================================
MY_DEVICE_TOKEN = '21d9ce6b-e764-4f0a-83a2-ed2bfdea09f6'

# Data Collection Trigger =====================================================
DATA_COLLECTION = False

# Defines =====================================================================
START_POS_X = 450
START_POS_Y = 450
WINDOW_HEIGHT = 1080
WINDOW_WIDTH = 1920
GRID_HEIGHT = 900
GRID_WIDTH = 900

SUPER_SHORT_SLEEP = 0.0001
SHORT_SLEEP = 0.001
MEDIUM_SLEEP = 0.1
LONG_SLEEP = 1
SUPER_LONG_SLEEP = 5

# Colour Definitions ==========================================================

BACKGROUND = '#18191A'
CARD = '#242526'
HOVER = '#3A3B3C'
PRIMARY_TEXT = '#E4E6EB'
SECONDARY_TEXT = '#B0B3B8'

# Classes =====================================================================

class TrackingData:
    """
    Class to hold the tracking data.
    """

    def __init__(self):
        self.node_ultra = [0] * 4
        self.accel = [0] * 3
        self.gyro = [0] * 3
        self.mag = [0] * 3
        self.delay = 0
        self.timestamp = 0
        self.node_rssi = [0] * 12
        self.node_distance = [450, 450, 450, 900, 950,
                              1000, 900, 1000, 900, 900, 600, 300]
        self.node_locations = [(0, 0), (300, 0), (600, 0), (900, 0),
                               (900, 300), (900, 600), (900, 900), (600, 900),
                               (300, 900), (0, 900), (0, 600), (0, 300)]
        self.fileList = ["datapoints" + str(i) + ".csv" for i in range(49)]
        self.currentFile = 0
        self.currentTestpoint = 0
        self.testxy = [0.5, 0.5]  # Make sure you do 49 different points
        self.node_transmit_power = [-35.5, -43.5, -39, -48.75, -51.75, -54.25,
                                    -48.5, -59, -52.25, -47.5, -44.5, -45.5]
        self.multilat_pos = (GRID_WIDTH/2, GRID_HEIGHT/2)
        self.kalman_pos = (GRID_WIDTH/2, GRID_HEIGHT/2)
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

        pos_x = self.testxy[0]
        pos_y = self.testxy[1]
        file_name = self.fileList[self.currentFile]

        rowDictionary = {'Pos_X': pos_x, 'Pos_Y': pos_y, 'Node_A': 0, 'Node_B': 0, 'Node_C': 0, 'Node_D': 0,
                         'Node_E': 0, 'Node_F': 0, 'Node_G': 0, 'Node_H': 0, 'Node_I': 0, 'Node_J': 0, 'Node_K': 0, 'Node_L': 0}

        fieldnames = ['Pos_X', 'Pos_Y', 'Node_A', 'Node_B', 'Node_C', 'Node_D', 'Node_E',
                      'Node_F', 'Node_G', 'Node_H', 'Node_I', 'Node_J', 'Node_K', 'Node_L']

        if self.currentTestpoint == 1:

            with open(file_name, 'w') as file:

                writer = csv.DictWriter(file, fieldnames=fieldnames)
                writer.writeheader()

                for i in range(12):
                    rowDictionary[fieldnames[i + 2]] = self.node_rssi[i]

                writer.writerow(rowDictionary)
                print("wrote")
        else:
            with open(file_name, 'a') as file:
                writer = csv.DictWriter(file, fieldnames=fieldnames)

                for i in range(12):
                    rowDictionary[fieldnames[i + 2]] = self.node_rssi[i]

                writer.writerow(rowDictionary)
                print("wrote: " + str(self.currentTestpoint))

    def rssi_to_distance(self):
        """
        Converts the received power (RSSI) data from dB to distance in m.
        :return: None
        """
        N = 4
        for idx, rssi in enumerate(self.node_rssi):
            if rssi != 0:
                self.node_distance[idx] = 10 ** (
                    (self.node_transmit_power[idx]-rssi) / (10*N))
            else:
                self.node_distance[idx] = 0

    def populate_data(self, raw_data):
        """
        Populates the tracking data with the raw data.
        :param raw_data: The raw data to populate the tracking data with.
        :return: None
        """
        self.node_ultra[0] = raw_data["Ultrasonic-1"]
        self.node_ultra[1] = raw_data["Ultrasonic-2"]
        self.node_ultra[2] = raw_data["Ultrasonic-3"]
        self.node_ultra[3] = raw_data["Ultrasonic-4"]
        self.accel[0] = raw_data["Accel-X"]
        self.accel[1] = raw_data["Accel-Y"]
        self.accel[2] = raw_data["Accel-Z"]
        self.gyro[0] = raw_data["Gyro-X"]
        self.gyro[1] = raw_data["Gyro-Y"]
        self.gyro[2] = raw_data["Gyro-Z"]
        self.mag[0] = raw_data["Mag-X"]
        self.mag[1] = raw_data["Mag-Y"]
        self.mag[2] = raw_data["Mag-Z"]
        self.timestamp = raw_data["Time-Stamp"]
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
        print("Ultrasonic: ", self.node_ultra)
        print("Accelerometer: ", self.accel)
        print("Gyro: ", self.gyro)
        print("Magnetometer: ", self.mag)
        print("Time: ", self.timestamp)
        print("Delay: ", self.delay)
        print("Node RSSI: ", self.node_rssi)
        print("Node Distance: ", self.node_distance)
        print("======================================================\n")

    def multilateration(self):
        """
        Use the least squares equation to estimate location of the object
        :return tuple: position (x,y)
        """

        fixed_node_x = []
        fixed_node_y = []
        fixed_node_distance = []

        # Remove nodes that have not provided an RSSI
        for idx, dist in enumerate(self.node_distance):
            if dist != 0:
                fixed_node_x.append(self.node_locations[idx][0])
                fixed_node_y.append(self.node_locations[idx][1])
                fixed_node_distance.append(dist)

        num_live_nodes = len(fixed_node_distance)

        x_fixed_array = np.array(fixed_node_x)
        y_fixed_array = np.array(fixed_node_y)
        radius_array = np.array(fixed_node_distance)

        BMat = np.array([(radius_array[i]**2 - radius_array[num_live_nodes - 1]**2-x_fixed_array[i]**2-y_fixed_array[i]
                         ** 2+x_fixed_array[num_live_nodes-1]**2 + y_fixed_array[num_live_nodes-1]**2) for i in range(num_live_nodes)])

        AMat = np.array([((2*(x_fixed_array[num_live_nodes - 1] - x_fixed_array[i])),
                         (2*(y_fixed_array[num_live_nodes - 1] - y_fixed_array[i]))) for i in range(num_live_nodes)])

        # Check case where an array is empty
        if len(AMat) == 0 or len(BMat) == 0:
            return

        FinalProd = np.linalg.lstsq(AMat, BMat, rcond=-1)[0]

        self.multilat_pos = FinalProd.tolist()

        self.multilat_pos[0] = math.ceil(self.multilat_pos[0])
        self.multilat_pos[1] = math.ceil(self.multilat_pos[1])

    def kalman_filter(self):
        """
        Use a Kalman filter to fuse the RF distance and US data to estimate the location of the object.
        :return tuple: position (x,y)
        """

class Kalman:
    def __init__(self, x_init, cov_init, meas_err, proc_err):
        self.ndim = len(x_init)
        self.A = np.array([(1, 0, 1, 0), (0, 1, 0, 1), (0, 0, 1, 0), (0, 0, 0, 1)])
        self.H = np.array([(1, 0, 0, 0), (0, 1, 0, 0)])
        self.x_hat =  x_init
        self.cov = cov_init
        self.Q_k = np.eye(self.ndim)*proc_err
        self.R = np.eye(len(self.H))*meas_err

    def update(self, obs):

        # Make prediction
        self.x_hat_est = np.dot(self.A,self.x_hat)
        self.cov_est = np.dot(self.A,np.dot(self.cov,np.transpose(self.A))) + self.Q_k

        # Update estimate
        self.error_x = obs - np.dot(self.H,self.x_hat_est)
        self.error_cov = np.dot(self.H,np.dot(self.cov_est,np.transpose(self.H))) + self.R
        self.K = np.dot(np.dot(self.cov_est,np.transpose(self.H)),np.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + np.dot(self.K,self.error_x)
        if self.ndim>1:
            self.cov = np.dot((np.eye(self.ndim) - np.dot(self.K,self.H)),self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est 

# GUI =========================================================================


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
        self._master.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self._master.configure(bg=BACKGROUND)
        self.pack()

        title = tk.Label(self._master, text="CSSE4011 GUI", borderwidth=0,
                         font="Montserrat, 25", bg=BACKGROUND, fg=PRIMARY_TEXT)
        title.pack(side=tk.TOP, padx=10, pady=10)

        # Create the grid
        self._grid = Grid(self._master)
        self._grid.place(relx=0.4, rely=0.5, anchor=tk.W)
        self._multilateration_node = MobileNode(self._grid, "multilateration")
        self._kalman_node = MobileNode(self._grid, "kalman")

        self._data_container = DataDisplayContainer(self._master)
        self._data_container.place(relx=0.3, rely=0.5, anchor=tk.E)
        self._data = DataDisplay(self._data_container)

        self._master.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.refresh_application()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self._stop = True
            self._master.destroy()

    def refresh_application(self):
        """
        Update the position of the mobile node and the values.
        """

        while True:
            time.sleep(SHORT_SLEEP)

            try:
                data = self.in_q.get(block=False)
            except Empty:
                data = None
            if data is not None:

                self._multilateration_node.target_x = data.multilat_pos[0]
                self._multilateration_node.target_y = data.multilat_pos[1]

                self._kalman_node.target_x = data.kalman_pos[0]
                self._kalman_node.target_y = data.kalman_pos[1]

                self._data.update_data(data)

            if self._stop:
                break
            # Update the GUI

            self._multilateration_node.redraw_position()
            self._kalman_node.redraw_position()
            self._master.update()


class Grid(tk.Canvas):
    """
    Class for the grid.
    """

    def __init__(self, master):
        super().__init__(master, bg=CARD, height=900, width=900, highlightthickness=0)
        self._master = master

        # create a grid
        for i in range(0, 900, 150):
            self.create_line(i, 0, i, 900, fill=SECONDARY_TEXT, width=2)
        for i in range(0, 900, 150):
            self.create_line(0, i, 900, i, fill=SECONDARY_TEXT, width=2)
        self.create_line(0, 0, 900, 0, fill=SECONDARY_TEXT, width=4)
        self.create_line(900, 0, 900, 900, fill=SECONDARY_TEXT, width=4)
        self.create_line(900, 900, 0, 900, fill=SECONDARY_TEXT, width=4)
        self.create_line(0, 900, 0, 0, fill=SECONDARY_TEXT, width=4)

        self.create_static_node_graphic(30, 30, 25)
        self.create_static_node_graphic(600-30, 30, 25)
        self.create_static_node_graphic(300-30, 30, 25)
        self.create_static_node_graphic(900-30, 30, 25)
        self.create_static_node_graphic(900-30, 600-30, 25)
        self.create_static_node_graphic(900-30, 300-30, 25)
        self.create_static_node_graphic(900-30, 900-30, 25)
        self.create_static_node_graphic(600-30, 900-30, 25)
        self.create_static_node_graphic(300-30, 900-30, 25)
        self.create_static_node_graphic(30, 900-30, 25)
        self.create_static_node_graphic(30, 600-30, 25)
        self.create_static_node_graphic(30, 300-30, 25)

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

            fill=SECONDARY_TEXT)


class DataDisplayContainer(tk.Canvas):

    def __init__(self, master):
        super().__init__(master, bg=CARD, height=900, width=450, highlightthickness=0)
        self._master = master


class DataDisplay(object):

    def __init__(self, canvas, master=None):
        self.canvas = canvas
        # Create Labels
        self.canvas.create_text(
            200, 30, text="Multilateration Position:", font="Montserrat, 12", fill="#E2703A", anchor="e")
        self.canvas.create_text(
            200, 50, text="Kalman Position:", font="Montserrat, 12", fill="#9C3D54", anchor="e")
        for idx in range(0, 12):
            self.canvas.create_text(
                200, 80 + idx*20, text="RSSI Node {}:".format(idx), font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        for idx in range(0, 12):
            self.canvas.create_text(
                200, 350 + idx*20, text="Distance Node {}:".format(idx), font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        for idx in range(0, 4):
            self.canvas.create_text(
                200, 610 + idx*20, text="Ultrasonic Distance {}:".format(idx), font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        self.canvas.create_text(
            200, 710, text="Accelerometer:", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        self.canvas.create_text(
            200, 740, text="Gyro:", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        self.canvas.create_text(
            200, 770, text="Magnetometer:", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        self.canvas.create_text(
            200, 800, text="Timestamp:", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")
        self.canvas.create_text(
            200, 830, text="Delay Time:", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e")

        # Create values
        self.multilat_pos = self.canvas.create_text(
            250, 30, text="NO DATA", font="Montserrat, 12", fill="#E2703A", anchor="w")
        self.kalman_pos = self.canvas.create_text(
            250, 50, text="NO DATA", font="Montserrat, 12", fill="#9C3D54", anchor="w")
        self.rssi = [0] * 12
        for idx in range(0, 12):
            self.rssi[idx] = self.canvas.create_text(
                250, 80 + idx*20, text="NO DATA".format(idx), font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.distance = [0] * 12
        for idx in range(0, 12):
            self.distance[idx] = self.canvas.create_text(
                250, 350 + idx*20, text="NO DATA".format(idx), font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.ultra = [0] * 4
        for idx in range(0, 4):
            self.ultra[idx] = self.canvas.create_text(
                250, 610 + idx*20, text="NO DATA".format(idx), font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.accel = self.canvas.create_text(
            250, 710, text="NO DATA", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.gyro = self.canvas.create_text(
            250, 740, text="NO DATA", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.mag = self.canvas.create_text(
            250, 770, text="NO DATA", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.time = self.canvas.create_text(
            250, 800, text="NO DATA", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")
        self.delay = self.canvas.create_text(
            250, 830, text="NO DATA", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="w")

    def update_data(self, data):
        """
        Redraw the position of the mobile node.
        """
        self.canvas.itemconfig(
            self.multilat_pos, text=f"({data.multilat_pos[0]}, {data.multilat_pos[1]})")
        self.canvas.itemconfig(
            self.kalman_pos, text=f"({data.kalman_pos[0]}, {data.kalman_pos[1]})")
        for idx in range(0, 12):
            self.canvas.itemconfig(
                self.rssi[idx], text=f"{data.node_rssi[idx]}")
        for idx in range(0, 12):
            self.canvas.itemconfig(
                self.distance[idx], text=f"{data.node_distance[idx]}")
        for idx in range(0, 4):
            self.canvas.itemconfig(
                self.ultra[idx], text=f"{data.node_ultra[idx]}")
        self.canvas.itemconfig(
            self.accel, text=f"({data.accel[0]}, {data.accel[1]}, {data.accel[2]})")
        self.canvas.itemconfig(
            self.gyro, text=f"({data.gyro[0]}, {data.gyro[1]}, {data.gyro[2]})")
        self.canvas.itemconfig(
            self.mag, text=f"({data.mag[0]}, {data.mag[1]}, {data.mag[2]})")
        self.canvas.itemconfig(self.time, text=f"{data.timestamp}")
        self.canvas.itemconfig(self.delay, text=f"{data.delay}")


class MobileNode(object):
    """
    Class for the mobile node
    """

    def __init__(self, canvas, type, master=None):
        self.current_x = START_POS_X
        self.current_y = START_POS_Y
        self.target_x = START_POS_X
        self.target_y = START_POS_Y
        self.canvas = canvas
        if type == "multilateration":
            self.node_colour = "#E2703A"
        elif type == "kalman":
            self.node_colour = "#9C3D54"

        self.graphic = self.canvas.create_oval(
            425, 425, 475, 475, fill=self.node_colour)
        self.text_position = self.canvas.create_text(
            450, 500, text="(500,500)", fill=PRIMARY_TEXT)

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
                ser.close()
                return


# Data Processing =============================================================


def data_processing(in_q, out_q, pub_q, stop):
    """
    Process the raw JSON data.
    """
    live_data = TrackingData()
    ndim = 4
    ndim_obs = 2
    xcoord = 5.0
    ycoord = 2.0
    vx = 0.5 #m.s
    vy = 1.0 #m/s
    dt = 1.0 #sec
    meas_error = 10.0 #m

    #generate ground truth
    x_true = np.array([xcoord,ycoord,vx,vy])
    obs_err = np.array([meas_error,meas_error])
    obs = x_true[0:1] + np.random.randn(ndim_obs)*obs_err

    #init filter
    proc_error = 0.01
    init_error = 150.0
    x_init = np.array( [xcoord+init_error, ycoord+init_error, vx, vy] ) #introduced initial xcoord error of 2m 
    cov_init=init_error*np.eye(ndim)
    x_hat = np.zeros((ndim))
    k_filter = Kalman(x_init, cov_init, meas_error, proc_error)
    
    while True:

        # Get the next message from the queue
        try:
            data_raw = in_q.get(block=False)
        except Empty:
            if stop():
                logging.info("Stoping Data Thread")
                break
            time.sleep(SHORT_SLEEP)
            continue
        now = datetime.now()  # Timestamp incomming data
        live_data.current_time = now.strftime("%H:%M:%S.%f")
        live_data.populate_data(data_raw)
        live_data.rssi_to_distance()
        live_data.multilateration()
        live_data.print_data()
        if DATA_COLLECTION:
            live_data.write_rssi_csv()

        # Process the data through the Kalman Filter
        k_filter.update([live_data.multilat_pos[0], live_data.multilat_pos[1]])
        live_data.kalman_pos = k_filter.x_hat

        # Send the estimated position to the GUI
        out_q.put(live_data)
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

    for idx, rssi in enumerate(live_data.node_rssi):

        publish_data.append(
            {
                'variable': 'rssi_' + str(idx + 1),
                'unit': 'dB',
                'value': rssi,
            }
        )

    for idx, ultra in enumerate(live_data.node_ultra):

        publish_data.append(
            {
                'variable': 'ultra_' + str(idx + 1),
                'unit': 'cm',
                'value': ultra,
            }
        )

    estimated_position = {
        "variable": "position",
        "value": 10,
        "metadata": {"x": live_data.kalman_pos[0]/900.0, "y": live_data.kalman_pos[1]/900.0},
    }

    publish_data.append(estimated_position)

    delay_time = {
        "variable": "delay",
        'unit': 'ms',
        "value": live_data.delay,
    }

    publish_data.append(delay_time)

    return publish_data


def MQTT_Publisher(pub_q, stop):

    my_device = tago.Device(MY_DEVICE_TOKEN)
    while(True):
        try:
            publish_data = pub_q.get(block=True, timeout=2)
        except Empty:
            if stop():
                logging.info("Stoping MQTT Thread")
                break
            continue

        result = my_device.insert(publish_data)

        if result['status']:
            logging.info(
                f"Successfully published with result: {result['result']}")
        else:
            logging.info(
                f"Fail to publish data with error:  {result['message']}")

        if stop():
            logging.info("Stoping MQTT Thread")
            break


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
