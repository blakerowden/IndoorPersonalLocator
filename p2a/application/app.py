"""
Prac 2a - Desktop Application
.py file 1/1
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden s4427634"

from cmath import sqrt
import time
import serial
import json
import tkinter as tk
from threading import Thread
from queue import *
import math


# GUI Classes =================================================================

class MainApplication(tk.Frame):
    """
    Main application class.
    """
    def __init__(self, inq_q, master=None):
        super().__init__(master)
        self._master = master
        self.in_q = inq_q
        self._master.title("Prac 2 - Position GUI Application")
        self._master.geometry("1050x1050")
        self._master.configure(bg="white")
        self.pack()

        # Create the grid
        self._grid = Grid(self._master)
        self._grid.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        self._mobile = mobile_node(self._grid)

        # Create the mobile node
        update_thread = Thread(target=self.update_position)
        update_thread.start()

    def update_position(self):
        """
        Update the position of the mobile node.
        """
        while True:
            # Get the next message from the queue
            #msg = self.in_q.get()

            # Update the position of the mobile node
            self._mobile.target_x = 500
            self._mobile.target_y = 500
            self._mobile.update_position()

            # Update the GUI
            self._master.update()
        

class mobile_node(object):
    """
    Class for the mobile node
    """
    def __init__(self, canvas, master = None):
        self.current_x = 500
        self.current_y = 500
        self.target_x = 500
        self.target_y = 500
        self.canvas = canvas

        self.graphic = self.canvas.create_oval(
                         475, 475, 525, 525, fill = "#afbecc")
        self.text_position = self.canvas.create_text(
                         500, 540, text="(500,500)", fill = "black")

    def update_position(self):
        if self.current_x != self.target_x or self.current_y != self.target_y:

            self.canvas.move(self.graphic, self.target_x - self.current_x, 
                    self.target_y - self.current_y)
            self.canvas.move(self.text_position, self.target_x - self.current_x, 
                    self.target_y - self.current_y)
            self.current_x = self.target_x
            self.current_y = self.target_y
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

def serial_interface(out_q):
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
    
    while(ser.is_open):
        line = serial_read_line(ser)
        if line:
            out_q.put(line)
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

def data_processing(in_q, out_q):
    """
    Process the raw JSON data.
    """
    while True:
        try:
            in_q.get(timeout=1)
            out_q.put(json.loads(in_q.get()))
        except Empty:
            pass

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
    comms_active = False

    j_data = Queue()    # Queue for JSON data
    k_data = Queue()    # Queue for (k)lean data

    if comms_active:
        
        # Create thread to read from the serial port
        thread_serial = Thread(target=serial_interface, args=(j_data,))
        thread_serial.start()

        # Create thread to process the data
        thread_data = Thread(target=data_processing, args=(j_data, k_data))
        thread_data.start()

    # Create thread to run the GUI
    thread_gui = Thread(target=gui_interface, args=(k_data,))
    thread_gui.start()

    Thread.join


if __name__ == "__main__":
    """
    Only run the main function if this file is run directly.
    """
    main()
