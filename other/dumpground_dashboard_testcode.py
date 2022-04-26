
import random
import time

import threading
import tago

MY_DEVICE_TOKEN = '82fcf5d7-cc97-44b0-89d4-46a9100dd88a'
my_device = tago.Device(MY_DEVICE_TOKEN)

rssi_10 = {
    'variable': 'rssi_10',
    'unit': 'dB',
    'value': 0
}
rssi_11 = {
    'variable': 'rssi_11',
    'unit': 'dB',
    'value': 0
}
rssi_12 = {
    'variable': 'rssi_12',
    'unit': 'dB',
    'value': 0
}
rssi_13 = {
    'variable': 'rssi_13',
    'unit': 'dB',
    'value': 0
}

rssi_14 = {
    'variable': 'rssi_14',
    'unit': 'dB',
    'value': 0
}
rssi_15 = {
    'variable': 'rssi_15',
    'unit': 'dB',
    'value': 0
}
rssi_16 = {
    'variable': 'rssi_16',
    'unit': 'dB',
    'value': 0
}
rssi_17 = {
    'variable': 'rssi_17',
    'unit': 'dB',
    'value': 0
}


rssi_20 = {
    'variable': 'rssi_20',
    'unit': 'dB',
    'value': 0
}
rssi_21 = {
    'variable': 'rssi_21',
    'unit': 'dB',
    'value': 0
}
rssi_22 = {
    'variable': 'rssi_22',
    'unit': 'dB',
    'value': 0
}
rssi_23 = {
    'variable': 'rssi_23',
    'unit': 'dB',
    'value': 0
}

rssi_24 = {
    'variable': 'rssi_24',
    'unit': 'dB',
    'value': 0
}
rssi_25 = {
    'variable': 'rssi_25',
    'unit': 'dB',
    'value': 0
}

rssi_26 = {
    'variable': 'rssi_26',
    'unit': 'dB',
    'value': 0
}

rssi_27 = {
    'variable': 'rssi_27',
    'unit': 'dB',
    'value': 0
}

us_10 = {
    'variable': 'us_10',
    'unit': 'm',
    'value': 0
}
us_11 = {
    'variable': 'us_11',
    'unit': 'm',
    'value': 0
}
us_12 = {
    'variable': 'us_12',
    'unit': 'm',
    'value': 0
}
us_13 = {
    'variable': 'us_13',
    'unit': 'm',
    'value': 0
}

us_20 = {
    'variable': 'us_20',
    'unit': 'm',
    'value': 0
}
us_21 = {
    'variable': 'us_21',
    'unit': 'm',
    'value': 0
}
us_22 = {
    'variable': 'us_22',
    'unit': 'm',
    'value': 0
}
us_23 = {
    'variable': 'us_23',
    'unit': 'm',
    'value': 0
}


us_node1 = {

    'variable': 'us_node1',
    'unit': 'cm',
    'value': [56, 34, 12, 45, 67]
}

us_node2 = {

    'variable': 'us_node2',
    'unit': 'cm',
    'value': [56, 34, 12, 45, 67]
}


rssi_node1 = {

    'variable': 'rssi_node1',
    'unit': 'dB',
    'value': [-56, -32, -45, -89, -36, -32, -45, -89]
}


rssi_node2 = {
    # Press the green button in the gutter to run the script.
    'variable': 'rssi_node2',
    'unit': 'dB',
    'value': [-56, -32, -45, -89, -36, -32, -45, -89]
}

list_data = []

list_data.append(rssi_10)
list_data.append(rssi_11)
list_data.append(rssi_12)
list_data.append(rssi_13)

list_data.append(rssi_14)
list_data.append(rssi_15)
list_data.append(rssi_16)
list_data.append(rssi_17)

list_data.append(rssi_20)
list_data.append(rssi_21)
list_data.append(rssi_22)
list_data.append(rssi_23)

list_data.append(rssi_24)
list_data.append(rssi_25)
list_data.append(rssi_26)
list_data.append(rssi_27)


list_data.append(us_10)
list_data.append(us_11)
list_data.append(us_12)
list_data.append(us_13)

list_data.append(us_20)
list_data.append(us_21)
list_data.append(us_22)
list_data.append(us_23)

list_data.append(us_node1)
list_data.append(us_node2)
list_data.append(rssi_node1)
list_data.append(rssi_node2)


class DashBoard:

    def __init__(self, list_data=list_data):

        self._list_data = list_data
        self._rssi_10 = list_data[0]
        self._rssi_11 = list_data[1]
        self._rssi_12 = list_data[2]
        self._rssi_13 = list_data[3]
        self._rssi_14 = list_data[4]
        self._rssi_15 = list_data[5]
        self._rssi_16 = list_data[6]
        self._rssi_17 = list_data[7]

        self._rssi_20 = list_data[8]
        self._rssi_21 = list_data[9]
        self._rssi_22 = list_data[10]
        self._rssi_23 = list_data[11]
        self._rssi_24 = list_data[12]
        self._rssi_25 = list_data[13]
        self._rssi_26 = list_data[14]
        self._rssi_27 = list_data[15]

        self._us_10 = list_data[16]
        self._us_11 = list_data[17]
        self._us_12 = list_data[18]
        self._us_13 = list_data[19]

        self._us_20 = list_data[20]
        self._us_21 = list_data[21]
        self._us_22 = list_data[22]
        self._us_23 = list_data[23]

        self._us_node1 = list_data[24]
        self._us_node2 = list_data[25]
        self._rssi_node1 = list_data[26]
        self._rssi_node2 = list_data[27]

        self._thread_handler = threading.Thread(target=self.start_publish)

    def start_publish(self):
        """continuously publishing data to the dashboard"""

        print("Thread starts")
        result = my_device.insert(list_data)  # With response
        if result['status']:
            print("Successfull starting publishing with result: ",
                  result['result'])
        else:
            print("Fail to publish data with error: ", result['message'])
        status = result['status']
        while status:
            result = my_device.insert(list_data)  # With response
            status = result['status']

            time.sleep(1)

    def start(self):
        """Start publishing data to the dashboard"""
        self._thread_handler.start()

    def stop(self):
        """Stop publishing data to the dashboard"""
        self._thread_handler.stop()

    def publish(self, ultra_data1, ultra_data2, rssi_data1, rssi_data2):
        """Update the dashboard with new data.

            Params:
                ultra_data1: A list of us values from team 1, eg: [89, 90]
                ultra_data2: A list of us values from team 2, eg: [84, 30]
                rssi_data1: A list of rssi values from team 1, eg: [-56, -32, -45,-89]
                rssi_data2: A list of rssi values from team 1, eg: [-46, -32, -25,-89]
        """

        self._rssi_10['value'] = rssi_data1[0]
        self._rssi_11['value'] = rssi_data1[1]
        self._rssi_12['value'] = rssi_data1[2]
        self._rssi_13['value'] = rssi_data1[3]
        self._rssi_14['value'] = rssi_data1[4]
        self._rssi_15['value'] = rssi_data1[5]
        self._rssi_16['value'] = rssi_data1[6]
        self._rssi_17['value'] = rssi_data1[7]

        self._rssi_20['value'] = rssi_data2[0]
        self._rssi_21['value'] = rssi_data2[1]
        self._rssi_22['value'] = rssi_data2[2]
        self._rssi_23['value'] = rssi_data2[3]
        self._rssi_24['value'] = rssi_data2[4]
        self._rssi_25['value'] = rssi_data2[5]
        self._rssi_26['value'] = rssi_data2[6]
        self._rssi_27['value'] = rssi_data2[7]

        self._us_10['value'] = ultra_data1[0]
        self._us_11['value'] = ultra_data1[1]
        self._us_12['value'] = ultra_data1[2]
        self._us_13['value'] = ultra_data1[3]

        self._us_20['value'] = ultra_data2[0]
        self._us_21['value'] = ultra_data2[1]
        self._us_22['value'] = ultra_data2[2]
        self._us_23['value'] = ultra_data2[3]

        self._us_node1['value'] = ultra_data1
        self._us_node2['value'] = ultra_data2
        self._rssi_node1['value'] = rssi_data1
        self._rssi_node2['value'] = rssi_data2


if __name__ == '__main__':

    dashboard = DashBoard()
    dashboard.start_publish()

    # t1.start() #start our thread
    # t1.join()
