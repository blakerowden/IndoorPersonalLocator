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
import collections
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import accuracy_score
from sklearn.impute import SimpleImputer
import pandas as pd
from sklearn.model_selection import cross_val_predict

DATAPATH = str(Path(__file__).parent / "Datapoints/datapoints")

def compile_data(loc_list, rssi_list, file_read):
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
                    loc_list.append(row[:2])
                    rssi_list.append(row[2:])
            i += 1

def predict_pos(rssi_list):

    fileKNN = DATAPATH + 'KNN' + ".csv"

    rssi_list = []
    loc_list = []
    entire_iteration = 0
    iteration = 1
    class_list = []

    compile_data(loc_list, rssi_list, fileKNN)

    for row in loc_list:
        if len(row) > 1:
            class_list.append(str(row[0]) + ',' + str(row[1]))

    X_train, X_test, Y_train, Y_test = train_test_split(rssi_list, class_list, test_size = .3, random_state = 4)

    knn = KNeighborsClassifier(n_neighbors = 10)

    knn.fit(X_train, Y_train)
    predictions = knn.predict([rssi_list])
    return predictions


