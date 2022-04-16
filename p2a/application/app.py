"""
Prac 2a - Desktop Application
.py file 1/1
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden s4427634"

import time
import serial
import json
import tkinter as tk
from threading import Thread
from queue import *

# GUI Classes =================================================================

class MainApplication(tk.Frame):
    """
    Main application class.
    """
    def __init__(self, master=None):
        super().__init__(master)
        self._master = master
        self._master.title("Prac 2a - Desktop Application")
        self._master.geometry("1000x1000")
        self._master.configure(bg="white")
        self.pack()
        self.create_widgets()

    def create_widgets(self):
        """
        Create the widgets for the application.
        """
        self.hi_there = tk.Button(self)
        self.hi_there["text"] = "Hello World\n(click me)"
        self.hi_there["command"] = self.say_hi
        self.hi_there.pack(side="top")

        self.quit = tk.Button(self, text="QUIT", fg="red",
                              command=self.master.destroy)
        self.quit.pack(side="bottom")

    def say_hi(self):
        """
        Print a message to the console.
        """
        print("hi there, everyone!")

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
    app = MainApplication(master=root)

    root.mainloop()

# Insertion Point =============================================================

def main():
    """
    Main function for the application.
    """
    j_data = Queue()    # Queue for JSON data
    k_data = Queue()    # Queue for (k)lean data

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
