"""
Prac 2a - Desktop Application
.py file 3/6
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden, Boston O'Neill and Liana can Teijlingen"

import csv
import math
from global_ import *
from dataclasses import dataclass
import numpy as np
from datetime import datetime
import time
from queue import *
from mqtt import *
import logging

# Data Collection Trigger =====================================================
DATA_COLLECTION = True

# Classes =====================================================================


@dataclass
class MobileNodeTrackingData:
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
        self.testxy = [0.5, 0.5]
        self.node_transmit_power = [-35.5, -43.5, -39, -48.75, -51.75, -54.25,
                                    -48.5, -59, -52.25, -47.5, -44.5, -45.5]
        self.multilat_pos = (450, 450)
        self.kalman_pos = (450, 450)
        self.rssi_error = 0
        self.us_error = 0
        self.current_time = ""

    def write_rssi_csv(self, is_meter_meas):
        """
        Writes the rssi data to a csv file.
        :param file_name: The name of the file to write to.
        :return: None
        """
        if(is_meter_meas == 1):
            # Change nodeName for ML:
            nodeName = '4011B'
            rowDictionary = {'Node': nodeName, 'RSSI': 0}
            fieldnames = ['Node', 'RSSI']
            file_name = 'datapoints' + nodeName + '.csv'
            nodeDictionary = {'4011A': 0, '4011B': 1, '4011C': 2, '4011D': 3, '4011E': 4,
                              '4011F': 5, '4011G': 6, '4011H': 7, '4011I': 8, '4011J': 9, '4011K': 10, '4011L': 11}
            if self.currentTestpoint == 501:
                return
            else:
                self.currentTestpoint += 1

            if self.currentTestpoint == 1:
                with open(file_name, 'w') as file:
                    writer = csv.DictWriter(file, fieldnames=fieldnames)
                    writer.writeheader()
                    rowDictionary[fieldnames[1]
                                  ] = self.node_rssi[nodeDictionary[nodeName]]
                    writer.writerow(rowDictionary)
                    print("wrote")
            else:
                with open(file_name, 'a') as file:
                    writer = csv.DictWriter(file, fieldnames=fieldnames)
                    rowDictionary[fieldnames[1]
                                  ] = self.node_rssi[nodeDictionary[nodeName]]
                    writer.writerow(rowDictionary)
                    print("wrote: " + str(self.currentTestpoint))

        else:
            # Change testxy for ML:
            self.testxy = [0.5, 0.5]
            pos_x = self.testxy[0]
            pos_y = self.testxy[1]

            if self.currentTestpoint == 501:
                return
            else:
                self.currentTestpoint += 1

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
        self.A = np.array([(1, 0, 1, 0), (0, 1, 0, 1),
                          (0, 0, 1, 0), (0, 0, 0, 1)])
        self.H = np.array([(1, 0, 0, 0), (0, 1, 0, 0)])
        self.x_hat = x_init
        self.cov = cov_init
        self.Q_k = np.eye(self.ndim)*proc_err
        self.R = np.eye(len(self.H))*meas_err

    def update(self, obs):

        # Make prediction
        self.x_hat_est = np.dot(self.A, self.x_hat)
        self.cov_est = np.dot(self.A, np.dot(
            self.cov, np.transpose(self.A))) + self.Q_k

        # Update estimate
        self.error_x = obs - np.dot(self.H, self.x_hat_est)
        self.error_cov = np.dot(self.H, np.dot(
            self.cov_est, np.transpose(self.H))) + self.R
        self.K = np.dot(np.dot(self.cov_est, np.transpose(
            self.H)), np.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + np.dot(self.K, self.error_x)
        if self.ndim > 1:
            self.cov = np.dot(
                (np.eye(self.ndim) - np.dot(self.K, self.H)), self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est

# Entry Point =================================================================


def data_processing_thread(in_q, out_q, pub_q, stop):
    """
    Process the raw JSON data.
    """
    live_data = MobileNodeTrackingData()
    ndim = 4
    ndim_obs = 2
    xcoord = 5.0
    ycoord = 2.0
    vx = 0.5  # m.s
    vy = 1.0  # m/s
    dt = 1.0  # sec
    meas_error = 10.0  # m

    # generate ground truth
    x_true = np.array([xcoord, ycoord, vx, vy])
    obs_err = np.array([meas_error, meas_error])
    obs = x_true[0:1] + np.random.randn(ndim_obs)*obs_err

    # init filter
    proc_error = 0.01
    init_error = 150.0
    # introduced initial xcoord error of 2m
    x_init = np.array([xcoord+init_error, ycoord+init_error, vx, vy])
    cov_init = init_error*np.eye(ndim)
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
            live_data.write_rssi_csv(1)

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
