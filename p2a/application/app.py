"""
Prac 2a - Desktop Application
.py file 1/1
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden s4427634"

from cmath import sqrt
import time
from turtle import update
import serial
import json
import tkinter as tk
from tkinter import messagebox
from threading import Thread
from queue import *
import math
import logging

# Defines =====================================================================
START_POS_X = 500
START_POS_Y = 500

SUPER_SHORT_SLEEP = 0.001
SHORT_SLEEP = 0.01
MEDIUM_SLEEP = 0.1
LONG_SLEEP = 1
SUPER_LONG_SLEEP = 10

# GUI Classes =================================================================

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
        self._master.geometry("1050x1050")
        self._master.configure(bg="white")
        self.pack()

        # Create the grid
        self._grid = Grid(self._master)
        self._grid.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        self._mobile = MobileNode(self._grid)
        self._master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Run the program
        self.update_position()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self._stop = True
            time.sleep(SHORT_SLEEP) # Allow time for the thread to close
            self._master.destroy()

    def update_position(self):
        """
        Update the position of the mobile node.
        """
        time.sleep(MEDIUM_SLEEP)
        while True:
            # Get the next message from the queue
    
            #with self.in_q.mutex:
            # Update the position of the mobile node
            try:
                pos = self.in_q.get(block = False)
            except Empty:
                pos = None
            if pos is not None:
                self._mobile.target_x = pos[0]
                self._mobile.target_y = pos[1]
                self._mobile.animate_movement()

            # Update the GUI
            self._master.update()

            if self._stop:
                break
                

class MobileNode(object):
    """
    Class for the mobile node
    """
    def __init__(self, canvas, master = None):
        self.current_x = START_POS_X
        self.current_y = START_POS_Y
        self.target_x =  START_POS_X
        self.target_y = START_POS_Y
        self.canvas = canvas

        self.graphic = self.canvas.create_oval(
                         475, 475, 525, 525, fill = "#afbecc")
        self.text_position = self.canvas.create_text(
                         500, 540, text="(500,500)", fill = "black")

    def animate_movement(self):
        """
        Animate the mobile node moving to the target position.
        """
        while self.current_x != self.target_x or self.current_y != self.target_y:
            self.redraw_position()
            time.sleep(SUPER_SHORT_SLEEP)
        # Redraw a final time to show final position
        self.redraw_position()

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
        super().__init__(master, bg = 'white', height=1000, width=1000)
        self._master = master
        
        # create a grid
        for i in range(0,1000,125):
            self.create_line(i,0,i,1000, fill="black")
        for i in range(0,1000,125):
            self.create_line(0,i,1000,i, fill="black")
        
        self.create_static_node_graphic(25, 25, 25)
        self.create_static_node_graphic(1000-25, 1000-25, 25)
        self.create_static_node_graphic(25, 1000-25, 25)
        self.create_static_node_graphic(1000-25, 25, 25)

    def create_static_node_graphic(self, pos_x, pos_y, size):
        """
        Create the graphic for the static node.
        """
        self.create_polygon(
            pos_x + size, pos_y, # Vertex A
            pos_x + (size/2), pos_y + math.sqrt(3)*size/2, # Vertex B
            pos_x - (size/2), pos_y + math.sqrt(3)*size/2, # Vertex B
            pos_x - size, pos_y, # Vertex D
            pos_x - (size/2), pos_y - math.sqrt(3)*size/2, # Vertex E
            pos_x + (size/2), pos_y - math.sqrt(3)*size/2, # Vertex F
        
            fill = "black")

# Serial Interface ============================================================

def serial_interface(out_q, stop):
    """
    Thread for the serial interfacing.
    """
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    ser.is_open = True
    print(f"Connected to Serial Port {ser.name}")
    time.sleep(SHORT_SLEEP)
    
    while(ser.is_open):
        line = serial_read_line(ser)
        if line:
            out_q.put(line)
            print(f"{line}\n")
        if stop():
            break
    ser.close()

def serial_read_line(ser):
    """
    Read a line from the serial port.
    """
    while(ser.is_open):
        line = ser.read(0xFFFF).decode('utf-8')[:-2].strip()
        if line:
            return line

# Data Processing =============================================================

def data_processing(in_q, out_q, stop):
    """
    Process the raw JSON data.
    """
    x_pos = START_POS_X
    y_pos = START_POS_Y
    
    while True:
        # Get the next message from the queue
        try:
            data_raw = in_q.get(block = False)
        except Empty:
            data_raw = None
        if data_raw is not None:
            data = json.loads(data_raw)
            out_q.queue.clear()
            out_q.put((x_pos, y_pos))    
            time.sleep(SHORT_SLEEP)
        if stop():
            logging.info("Stoping Data Thread")
            break


# GUI Interface ===============================================================

def gui_interface(in_q):
    """
    GUI interface for the application.
    """
    root = tk.Tk()
    app = MainApplication(in_q, master=root)

    root.mainloop()

# Insertion Point =============================================================

def main():
    """
    Main function for the application.
    """
    # Set logging level
    logging.basicConfig(level=logging.DEBUG)
    # Create a stop flag
    stop_flag = False
    comms_active = False
    data_active = True
    gui_active = True

    j_data = Queue()    # Queue for JSON data
    k_data = Queue()    # Queue for (k)lean data

    if comms_active:
        # Create thread to read from the serial port
        thread_serial = Thread(target=serial_interface, args=(j_data, lambda: stop_flag))
        thread_serial.start()

    if data_active:
        # Create thread to process the data
        thread_data = Thread(target=data_processing, args=(j_data, k_data, lambda: stop_flag))
        thread_data.start()

    if gui_active:
        # Create thread to run the GUI
        thread_gui = Thread(target=gui_interface, args=(k_data,))
        thread_gui.start()

    while not stop_flag:
        if not thread_gui.is_alive():
            stop_flag = True
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


#TO-DO:
# - Take in ranging data from the serial port 
#       Ultrasonic x2, RSSI, Displacement and Heading (IMU) and Timestamp 
#       Process in JSON File
# - Import known static beacon locations create matrix A and vector
#    b of the linearised multilateration problem
# - Use the least squares equation to estimate location of the object. Refer to Tutorial 8,
#    which introduces the Numpy Least Squares Function.
# - Create a GUI to display the location of the object.
#Estimate the error of RF and ultrasound ranging methods based on measuring the
#ranging error at several different Tag locations.
#• Develop a localization system that uses both RF (high coverage, low accuracy) and
#ultrasound (low coverage, high accuracy) methods. Use a Kalman filter to fuse information from the IMU and RF, ultrasonic localization methods and track a moving
#object. You can assume a simple motion model (constant velocity motion). You can
#use the motion and heading information from the mobile node. You can assume the
#starting position of the tracked tag is known. You must be able display the location estimates from the Kalman filter output (graphical UI) and demonstrate higher
#localisation accuracy in areas where ultrasound ranging is available.