import serial
import time
import re

import paho.mqtt.client as mqtt
import argparse

def publish(client, topic, message):
    client.publish(topic, message)

def on_log(client, userdata, level, buf):
    print ("log: " + buf)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print ("connected OK")
    else:
        print ("Bad connection returned code =", rc)

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection.")

def on_message(client, userdata, message):
    print (message.payload)

def main(args):
    client = mqtt.Client("client1")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
        
    if args.verbose:
        client.on_log = on_log
    
    print ("connecting to broker", args.host)
    client.connect('localhost', 1883)
    time.sleep(2)
    client.subscribe('testtopic')
    
    print("subscribed")
    time.sleep(5)
    ser = serial.Serial("/dev/ttyACM0")
    time.sleep(5)
    ser.write(str.encode("all o\r"))
    ser.write(str.encode("sample w 5\r"))
    
    while True:
        reader = 0
        time.sleep(0.05)
        reading = ser.readline().decode('utf-8')
        if ('0x' in reading):
            print("Publishing: ", reading)
            publish(client, 'testtopic', reading)
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', action='store_true', dest='verbose', required=False)
    parser.add_argument('-H', action='store', dest='host', required=False, default="localhost")
    parser.add_argument('-p', action='store', dest='port', type=int, required=False, default="1883")
    parser.add_argument('-t', action='store', dest='topic', required=True)
    args = parser.parse_args()

    main(args)