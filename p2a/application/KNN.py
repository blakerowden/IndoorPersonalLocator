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

DATAPATH = str(Path(__file__).parent / "Datapoints/datapoints")


def compile_data(locations, rssi_list, file_read):
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
                locations.append(row[:2])
                rssi_list.append(row[2:])
            i += 1


files = [DATAPATH + str(i) + ".csv" for i in range(49)]

loc_list = []
rssi_list = []

for i in range(49):
    compile_data(loc_list, rssi_list, files[i])



