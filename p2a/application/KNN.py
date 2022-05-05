"""
Prac 2a - Desktop Application
.py file 6/7 - Machine Learning
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "B.O'Neill"

import csv
from global_ import *
from queue import *
from mqtt import *
from pathlib import Path
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier

DATAPATH = str(Path(__file__).parent / "Datapoints/combined")


def compile_data(loc_list: list, rssi_list: list, file_read: str) -> None:
    """Compile the data from the csv file into a list of lists

    Args:
        loc_list (list): List of lists to store the location data
        rssi_list (list): List of lists to store the rssi data
        file_read (str): The file to read from
    """
    with open(file_read, "r") as file:
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

        csv_row = {
            "Pos_X": 0,
            "Pos_Y": 0,
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
        csv_reader = csv.reader(file, delimiter=",")
        lineCount = 0
        i = 0
        for row in csv_reader:
            if i != 0:
                if len(row) > 1:
                    loc_list.append(row[:1])
                    rssi_list.append(row[1:])
            i += 1


def predict_pos(rssi_input_list: list) -> tuple:
    """Predict the position of the device based on the rssi data

    Args:
        rssi_list_2 (list): List of lists to store the rssi data

    """

    indexToCoord = [
        (0, 0),
        (0, 133),
        (0, 266),
        (0, 400),
        (133, 0),
        (133, 133),
        (133, 266),
        (133, 400),
        (266, 0),
        (266, 133),
        (266, 266),
        (266, 400),
        (400, 0),
        (400, 133),
        (400, 266),
        (400, 400),
        (200, 200),
    ]

    fileKNN = DATAPATH + ".csv"

    rssi_list = []
    loc_list = []
    entire_iteration = 0
    iteration = 1
    class_list = []

    compile_data(loc_list, rssi_list, fileKNN)

    for row in loc_list:
        if len(row) > 0:
            class_list.append(str(row[0]))

    X_train, X_test, Y_train, Y_test = train_test_split(rssi_list, class_list)

    knn = KNeighborsClassifier(n_neighbors=8)

    knn.fit(X_train, Y_train)

    predictions = knn.predict([rssi_input_list])

    if int(predictions.tolist()[0]) < 17:
        return indexToCoord[int(predictions.tolist()[0])]
