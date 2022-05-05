"""
Prac 2a - Desktop Application
.py file 3/7 - Data Processing
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "B.Rowden, B.O'Neill and L.van Teijlingen"

import csv
import math
from global_ import *
import numpy as np
from datetime import datetime
import time
from queue import *
from mqtt import *
import logging
import random
from pathlib import Path
from KNN import predict_pos

# Data Management Defines =====================================================
TEST_POINT_X = 200  # position in cm
TEST_POINT_Y = 200  # position in cm
FILE_NO = "test0"
DATA_NODE_NAME = "4011A"
DATA_COLLECTION_ACTIVE = False

DATAPATH = str(Path(__file__).parent / "Datapoints/datapoints")
TOTAL_TEST_POINTS = 50
ONE_METER_POWER_MODE = False  # True = 1 Node/1m, False = All Nodes/ML Readings

DATA_SIMULATE = False  # Feeds simulation data to the data processing thread

# Defines =====================================================================
GRID_LENGTH_CM = 4_00  # 4m x 4m grid
GRID_LENGTH = GRID_LENGTH_CM
GRID_THIRD = GRID_LENGTH_CM / 3
GRID_HALF = GRID_LENGTH_CM / 2
GRID_TWO_THIRD = GRID_THIRD * 2

# Classes =====================================================================


class MobileNodeTrackingData:
    """
    Class to hold the tracking data.
    """

    def __init__(self):

        # Ultrasound
        self.node_ultra = [0] * 4
        self.node_ultra_locations = [
            (GRID_THIRD, 0),
            (GRID_LENGTH, GRID_THIRD),
            (GRID_TWO_THIRD, GRID_LENGTH),
            (0, GRID_TWO_THIRD),
        ]

        # IMU
        self.accel = [0] * 3
        self.gyro = [0] * 3
        self.mag = [0] * 3

        self.node_rssi = [0] * 12
        self.node_distance = [0] * 12
        self.node_locations = [
            (0, 0),
            (GRID_THIRD, 0),
            (GRID_TWO_THIRD, 0),
            (GRID_LENGTH, 0),
            (GRID_LENGTH, GRID_THIRD),
            (GRID_LENGTH, GRID_TWO_THIRD),
            (GRID_LENGTH, GRID_LENGTH),
            (GRID_TWO_THIRD, GRID_LENGTH),
            (GRID_THIRD, GRID_LENGTH),
            (0, GRID_LENGTH),
            (0, GRID_TWO_THIRD),
            (0, GRID_THIRD),
        ]

        self.node_transmit_power = [
            -35.17,
            -38.68,
            -40.50,
            -38.04,
            -35.84,
            -37.12,
            -42.88,
            -38.49,
            -37.12,
            -43.37,
            -39.25,
            -36.47,
        ]

        self.timestamp = 0
        self.last_timestamp = 0
        self.rssi_delay = 0
        self.packet_timelog = "0"
        self.epoch = 0
        self.packet_time_delta = 0

        # Calculated positions
        self.multilat_pos = (GRID_HALF, GRID_HALF)
        self.k_multilat_pos = (GRID_HALF, GRID_HALF)
        self.ultrasonic_pos = (GRID_HALF, GRID_HALF)
        self.fusion_pos = (GRID_HALF, GRID_HALF)
        self.ml_pos = (GRID_HALF, GRID_HALF)
        self.k_ml_pos = (GRID_HALF, GRID_HALF)

        self.rssi_error = 500  # RSSI error in cm
        self.us_error = 5  # Ultrasonic error in cm
        self.ml_error = 2  # ML error in cm

        self.training_data = [DATAPATH + "test" + str(i) + ".csv" for i in range(49)]
        self.training_data_selected = 0
        self.data_points_collected = 0

        self.initial_state_mean = np.array(
            [
                self.multilat_pos[0] + self.rssi_error,
                self.multilat_pos[1] + self.rssi_error,
                self.accel[0],
                self.accel[1],
            ]
        )

        self.initial_ml_state_mean = np.array(
            [
                self.multilat_pos[0] + self.ml_error,
                self.multilat_pos[1] + self.ml_error,
                0,
                0,
            ]
        )

        self.initial_state_covariance = self.rssi_error * np.eye(
            len(self.initial_state_mean)
        )

        self.initial_ml_state_covariance = self.ml_error * np.eye(
            len(self.initial_ml_state_mean)
        )

        self.multilat_k_filter = MultilateralKalman(
            self.initial_state_mean, self.initial_state_covariance, self.rssi_error
        )
        self.ml_k_filter = MultilateralKalman(
            self.initial_state_mean, self.initial_ml_state_covariance, self.ml_error
        )

    def write_rssi_csv(self):
        """
        Writes the rssi data to a csv file.
        :param file_name: The name of the file to write to.
        :return: None
        """
        if ONE_METER_POWER_MODE:
            # Change nodeName for ML:

            csv_field_names = ["Node", "RSSI"]
            csv_file_name = "datapoints" + DATA_NODE_NAME + ".csv"
            node_dictionary = {
                "4011A": 0,
                "4011B": 1,
                "4011C": 2,
                "4011D": 3,
                "4011E": 4,
                "4011F": 5,
                "4011G": 6,
                "4011H": 7,
                "4011I": 8,
                "4011J": 9,
                "4011K": 10,
                "4011L": 11,
            }
            csv_row = {"Node": DATA_NODE_NAME, "RSSI": node_dictionary[DATA_NODE_NAME]}
            if self.data_points_collected == TOTAL_TEST_POINTS:
                return
            else:
                self.data_points_collected += 1

            if self.data_points_collected == 1:
                with open(csv_file_name, "w") as file:
                    writer = csv.DictWriter(file, fieldnames=csv_field_names)
                    writer.writeheader()
                    csv_row[csv_field_names[1]] = self.node_rssi[
                        node_dictionary[DATA_NODE_NAME]
                    ]
                    writer.writerow(csv_row)
                    print("wrote")
            else:
                with open(csv_file_name, "a") as file:
                    writer = csv.DictWriter(file, fieldnames=csv_field_names)
                    csv_row[csv_field_names[1]] = self.node_rssi[
                        node_dictionary[DATA_NODE_NAME]
                    ]
                    writer.writerow(csv_row)
                    print("wrote: " + str(self.data_points_collected))

        else:
            pos_x = TEST_POINT_X
            pos_y = TEST_POINT_Y
            self.training_data_selected = FILE_NO
            if self.data_points_collected == 201:
                return
            else:
                self.data_points_collected += 1

            csv_file_name = "datapoints" + self.training_data_selected + ".csv"

            csv_row = {
                "Pos_X": pos_x,
                "Pos_Y": pos_y,
                "Node_A": 0,
                "Node_B": 0,
                "Node_C": 0,
                "Node_D": 0,
                "Node_E": 0,
                "Node_F": 0,
                "Node_G": 0,
                "Node_H": 0,
                "Node_I": 0,
                "Node_J": 0,
                "Node_K": 0,
                "Node_L": 0,
            }

            csv_field_names = [
                "Pos_X",
                "Pos_Y",
                "Node_A",
                "Node_B",
                "Node_C",
                "Node_D",
                "Node_E",
                "Node_F",
                "Node_G",
                "Node_H",
                "Node_I",
                "Node_J",
                "Node_K",
                "Node_L",
            ]

            if self.data_points_collected == 1:
                with open(csv_file_name, "w") as file:
                    writer = csv.DictWriter(file, fieldnames=csv_field_names)
                    writer.writeheader()
                    for i in range(12):
                        csv_row[csv_field_names[i + 2]] = self.node_rssi[i]
                    writer.writerow(csv_row)
                    print("wrote")
            else:
                with open(csv_file_name, "a") as file:
                    writer = csv.DictWriter(file, fieldnames=csv_field_names)
                    for i in range(12):
                        csv_row[csv_field_names[i + 2]] = self.node_rssi[i]
                    writer.writerow(csv_row)
                    print("wrote: " + str(self.data_points_collected))

    def random_RSSI(self, x: int, y: int) -> None:
        """
        Generates random RSSI values for a given position
        :return: None
        """

        for i in range(15):
            fileName = self.training_data[i]
            rowNum = random.randint(1, 50)

            with open(fileName) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=",")
                lineCount = 0
                for row in csv_reader:
                    if lineCount == rowNum and row[0] == str(x) and row[1] == str(y):
                        for i in range(12):
                            self.node_rssi[i] = (int)(row[i + 2])
                    lineCount += 1

    def rssi_to_distance(self) -> None:
        """
        Converts the received power (RSSI) data from dB to distance in cm.
        :return: None
        """
        N = 4
        for idx, rssi in enumerate(self.node_rssi):
            if rssi != 0:
                self.node_distance[idx] = (
                    10 ** ((self.node_transmit_power[idx] - rssi) / (10 * N))
                ) * 100
            else:
                self.node_distance[idx] = 0

    def populate_data(self, raw_data: dict):
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
        self.timestamp = raw_data["Time-Stamp"] - self.epoch
        self.rssi_delay = raw_data["Delay-Time"]
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

    def __str__(self) -> str:
        """
        Prints the recieved raw data to the console.
        :return: None
        """
        print_string = ""
        print_string += (
            f"======== Raw Data Recieved @ {self.packet_timelog} ======== \n"
        )
        print_string += f"Ultrasonic: {self.node_ultra} \n"
        print_string += f"Accelerometer: {self.accel} \n"
        print_string += f"Gyro: {self.gyro} \n"
        print_string += f"Magnetometer: {self.mag} \n"
        print_string += f"Time: {self.timestamp/100.00} s\n"
        print_string += f"Packet Delay: {self.packet_time_delta} ms \n"
        print_string += f"RSSI Delay: {self.rssi_delay} ms \n"
        print_string += f"Node RSSI: {self.node_rssi} \n \n"
        print_string += f"================= Calculated Data ===================== \n"
        print_string += f"Node Distance: {self.node_distance} \n"
        print_string += f""
        print_string += f"Multilateration Co-ord: {self.multilat_pos} \n"
        print_string += f"Kalman Co-ord: {self.k_multilat_pos} \n"
        print_string += f"======================================================== \n"
        return print_string

    def rssi_multilateration(self) -> None:
        """
        Use the least squares equation to estimate location of the object
        :return tuple: position (x,y)
        """

        fixed_node_x = []
        fixed_node_y = []
        fixed_node_distance = []

        # Remove nodes that have not provided an RSSI due to error
        for idx, dist in enumerate(self.node_distance):
            if dist != 0:
                fixed_node_x.append(float(self.node_locations[idx][0]))
                fixed_node_y.append(float(self.node_locations[idx][1]))
                fixed_node_distance.append(dist)

        # Check case where an array is empty
        if len(fixed_node_distance) < 6:
            return

        # Apply a weighting to the RSSI data
        distance, x_pos, y_pos = zip(
            *sorted(zip(fixed_node_distance, fixed_node_x, fixed_node_y))
        )
        distance = distance[:3]
        x_pos = x_pos[:3]
        y_pos = y_pos[:3]

        num_live_nodes = len(distance)

        x_fixed_array = np.array(x_pos)
        y_fixed_array = np.array(y_pos)
        radius_array = np.array(distance)
        vec_b = []
        # Calculate the least squares equation
        for i in range(num_live_nodes):
            vec_b.append(
                (
                    radius_array[i] ** 2
                    - radius_array[num_live_nodes - 1] ** 2
                    - x_fixed_array[i] ** 2
                    - y_fixed_array[i] ** 2
                    + x_fixed_array[num_live_nodes - 1] ** 2
                    + y_fixed_array[num_live_nodes - 1] ** 2
                )
            )

        vec_b = np.array(vec_b)

        mat_a = []

        for i in range(num_live_nodes):
            mat_a.append(
                (
                    (2 * (x_fixed_array[num_live_nodes - 1] - x_fixed_array[i])),
                    (2 * (y_fixed_array[num_live_nodes - 1] - y_fixed_array[i])),
                )
            )

        mat_a = np.array(mat_a)

        lst_sq_estimate = np.linalg.lstsq(mat_a, vec_b, rcond=-1)[0]

        return_pos_x, return_pos_y = lst_sq_estimate.tolist()

        if return_pos_x == 0.0:
            return_pos_x = x_pos[0]
        if return_pos_y == 0.0:
            return_pos_y = y_pos[0]

        # Check for edge cases where the estimate is outside the range
        if return_pos_x <= 0:
            return_pos_x = 0 + random.randint(0, 50)
        elif return_pos_x >= GRID_LENGTH_CM:
            return_pos_x = GRID_LENGTH_CM - random.randint(0, 50)
        if return_pos_y <= 0:
            return_pos_y = 0 + random.randint(0, 50)
        elif return_pos_y >= GRID_LENGTH_CM:
            return_pos_y = GRID_LENGTH_CM - random.randint(0, 50)

        self.multilat_pos = (math.ceil(return_pos_x), math.ceil(return_pos_y))

    def kalman_filter(self) -> None:
        """
        Apply the Kalman filter to the data
        :return: None
        """
        lat_observation = np.array([self.multilat_pos[0], self.multilat_pos[1]])
        self.multilat_k_filter.predict()
        self.multilat_k_filter.update(lat_observation)
        self.k_multilat_pos = self.multilat_k_filter.get_state()

        ml_observation = np.array([self.ml_pos[0], self.ml_pos[1]])
        self.ml_k_filter.predict()
        self.ml_k_filter.update(ml_observation)
        self.k_ml_pos = self.ml_k_filter.get_state()

    def ultrasonic_position(self) -> None:
        """
        Calculate the estimated position from the ultrasonic sensors
        :return: None
        """
        position_estimates = []
        position_x = 0
        position_y = 0

        if self.node_ultra[0] <= 350 and self.node_ultra[0] != 0:
            position_estimates.append(
                (
                    self.node_ultra_locations[0][0],
                    self.node_ultra_locations[0][1] + self.node_ultra[0],
                )
            )

        if self.node_ultra[1] <= 350 and self.node_ultra[1] != 0:
            position_estimates.append(
                (
                    self.node_ultra_locations[1][0] - self.node_ultra[1],
                    self.node_ultra_locations[1][1],
                )
            )

        if self.node_ultra[2] <= 350 and self.node_ultra[2] != 0:
            position_estimates.append(
                (
                    self.node_ultra_locations[2][0],
                    self.node_ultra_locations[2][1] - self.node_ultra[2],
                )
            )

        if self.node_ultra[3] <= 350 and self.node_ultra[3] != 0:
            position_estimates.append(
                (
                    self.node_ultra_locations[3][0] + self.node_ultra[3],
                    self.node_ultra_locations[3][1],
                )
            )

        if len(position_estimates) == 0:
            self.ultrasonic_pos = (0, 0)
            return

        for pos in position_estimates:
            position_x += pos[0]
            position_y += pos[1]

        position_x = position_x / len(position_estimates)
        position_y = position_y / len(position_estimates)

        self.ultrasonic_pos = (position_x, position_y)
        print(self.ultrasonic_pos)

    def sensor_fusion(self) -> None:
        """
        Combine the sensor data to estimate the position of the object
        :return: None
        """
        x_position = 0
        y_position = 0

        p_weighting = 0.9  # Weighting of the position estimate for US
        self.rssi_to_distance()
        self.rssi_multilateration()
        self.kalman_filter()
        self.ultrasonic_position()

        if self.ultrasonic_pos[0] != 0:
            x_position += self.ultrasonic_pos[0] * p_weighting
        if self.ultrasonic_pos[1] != 0:
            y_position += self.ultrasonic_pos[1] * p_weighting

        if self.k_multilat_pos[0] != 0:
            if self.ultrasonic_pos[0] == 0:
                x_position += self.k_multilat_pos[0]
            else:
                x_position += self.k_multilat_pos[0] * (1 - p_weighting)
        if self.k_multilat_pos[1] != 0:
            if self.ultrasonic_pos[1] == 0:
                y_position += self.k_multilat_pos[1]
            else:
                y_position += self.k_multilat_pos[1] * (1 - p_weighting)

        self.k_multilat_pos = (x_position, y_position)


class MultilateralKalman:
    def __init__(self, x_0, P_0, meas_err):
        # Constants
        self.process_noise = 0.05
        self.ndim = len(x_0)
        self._dt = 143
        self._vx = 0.01
        self._vy = 0.01

        self.A = np.array(
            [
                (1, 0, self._dt, 0),
                (0, 1, 0, self._dt),
                (0, 0, self._vx, 0),
                (0, 0, 0, self._vy),
            ]
        )  # State-transition model
        self.H = np.array([(1, 0, 0, 0), (0, 1, 0, 0)])  # Observation model
        self.Q = (
            np.eye(self.ndim) * self.process_noise
        )  # Covariance of the process noise
        self.R = np.eye(len(self.H)) * meas_err  # Covariance of the observation noise

        # Initial state
        self._x = np.array(x_0)
        self._P = np.array(P_0)

    def predict(self):
        """Calculate the predicted state and covariance"""
        self._x = self.A @ self._x  # Predicted (a priori) state estimate
        self._P = (
            self.A @ self._P @ self.A.transpose() + self.Q
        )  # Predicted (a priori) estimate covariance

    def update(self, observation: np.array) -> None:
        """Update the state estimate based on the observation.

        Args:
            observation (np.array): The observation vector.
        """

        self.innovation_covariance = (
            self.H @ self._P @ self.H.transpose() + self.R
        )  # Innovation (or pre-fit residual) covariance
        self.observation_noise = (
            observation - self.H @ self._x
        )  # Innovation or measurement pre-fit residual
        self.kalman_gain = (
            self._P @ self.H.transpose() @ np.linalg.inv(self.innovation_covariance)
        )  # Optimal Kalman gain

        self._x = (
            self._x + self.kalman_gain @ self.observation_noise
        )  # Updated (a posteriori) state estimate
        self._P = (
            self._P
            - self.kalman_gain
            @ self.innovation_covariance
            @ self.kalman_gain.transpose()
        )  # Updated (a posteriori) estimate covariance

    def get_state(self) -> tuple:
        """Returns the current state estimate.

        Returns:
            tuple: Current co-ordinate estimate.
        """
        return self._x.tolist()


def data_processing_thread(
    raw_in_q: Queue, gui_out_q: Queue, mqtt_pub_q: Queue, stop
) -> None:
    """Thread to process the data from the sensors

    Args:
        raw_in_q (Queue): Queue to get the raw data from the sensors
        gui_out_q (Queue): Queue to send the processed data to the GUI
        mqtt_pub_q (Queue): Queue to send the processed data to the MQTT server
        stop (_type_): Stop flag
    """
    live_data = MobileNodeTrackingData()
    first_packet_received = False

    while True:

        # Get the next message from the queue_summary_
        try:
            data_raw = raw_in_q.get(block=False)
        except Empty:
            if stop():
                logging.info("Stoping Data Thread")
                break
            time.sleep(SHORT_SLEEP)
            continue

        now = datetime.now()  # Timestamp incomming data
        live_data.populate_data(data_raw)

        if not first_packet_received:
            first_packet_received = True
            logging.info("First packet received")
            live_data.epoch = live_data.timestamp
            live_data.timestamp = 0

        live_data.packet_time_delta = live_data.timestamp - live_data.last_timestamp
        live_data.last_timestamp = live_data.timestamp
        live_data.packet_timelog = now.strftime("%H:%M:%S.%f")

        if DATA_SIMULATE:
            live_data.random_RSSI(TEST_POINT_X, TEST_POINT_Y)

        live_data.sensor_fusion()

        if DATA_COLLECTION_ACTIVE:
            live_data.write_rssi_csv()

        prediction = predict_pos(live_data.node_rssi)

        if prediction:
            live_data.ml_pos = prediction
        # Send the estimated position to the GUI
        gui_out_q.queue.clear()
        gui_out_q.put(live_data)

        # Send the estimated position to the MQTT server
        mqtt_packet = MQTT_Packer(live_data)
        mqtt_pub_q.queue.clear()
        mqtt_pub_q.put(mqtt_packet)

        # Print the data to the console
        print(live_data)

        if stop():
            logging.info("Stoping Data Thread")
            break
