"""
Prac 2a - Desktop Application
.py file 4/6
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "Blake Rowden, Boston O'Neill and Liana can Teijlingen"

import tkinter as tk
from tkinter import messagebox
import time
from global_ import *
import math
from threading import Thread
from queue import *

# GUI Defines =================================================================

BACKGROUND = "#18191A"
CARD = "#242526"
HOVER = "#3A3B3C"
PRIMARY_TEXT = "#E4E6EB"
SECONDARY_TEXT = "#B0B3B8"

WINDOW_HEIGHT = 1080
WINDOW_WIDTH = 1920
GRID_HEIGHT = 900
GRID_WIDTH = 900

# GUI =========================================================================


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
        self._master.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self._master.configure(bg=BACKGROUND)
        self.pack()

        title = tk.Label(
            self._master,
            text="CSSE4011 GUI",
            borderwidth=0,
            font="Montserrat, 25",
            bg=BACKGROUND,
            fg=PRIMARY_TEXT,
        )
        title.pack(side=tk.TOP, padx=10, pady=10)

        # Create the grid
        self._grid = Grid(self._master)
        self._grid.place(relx=0.4, rely=0.5, anchor=tk.W)
        self._multilateration_node = MobileNode(self._grid, "multilateration")
        self._kalman_node = MobileNode(self._grid, "kalman")

        self._data_container = DataDisplayContainer(self._master)
        self._data_container.place(relx=0.3, rely=0.5, anchor=tk.E)
        self._data = DataDisplay(self._data_container)

        self._master.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.refresh_application()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self._stop = True
            self._master.destroy()

    def refresh_application(self):
        """
        Update the position of the mobile node and the values.
        """

        while True:
            time.sleep(SHORT_SLEEP)

            try:
                data = self.in_q.get(block=False)
            except Empty:
                data = None
            if data is not None:

                self._multilateration_node.target_x = data.multilat_pos[0]
                self._multilateration_node.target_y = data.multilat_pos[1]

                self._kalman_node.target_x = data.kalman_pos[0]
                self._kalman_node.target_y = data.kalman_pos[1]

                self._data.update_data(data)

            if self._stop:
                break
            # Update the GUI

            self._multilateration_node.redraw_position()
            self._kalman_node.redraw_position()
            self._master.update()


class Grid(tk.Canvas):
    """
    Class for the grid.
    """

    def __init__(self, master):
        super().__init__(master, bg=CARD, height=900, width=900, highlightthickness=0)
        self._master = master

        # create a grid
        for i in range(0, 900, 150):
            self.create_line(i, 0, i, 900, fill=SECONDARY_TEXT, width=2)
        for i in range(0, 900, 150):
            self.create_line(0, i, 900, i, fill=SECONDARY_TEXT, width=2)
        self.create_line(0, 0, 900, 0, fill=SECONDARY_TEXT, width=4)
        self.create_line(900, 0, 900, 900, fill=SECONDARY_TEXT, width=4)
        self.create_line(900, 900, 0, 900, fill=SECONDARY_TEXT, width=4)
        self.create_line(0, 900, 0, 0, fill=SECONDARY_TEXT, width=4)

        self.create_static_node_graphic(30, 30, 25)
        self.create_static_node_graphic(600 - 30, 30, 25)
        self.create_static_node_graphic(300 - 30, 30, 25)
        self.create_static_node_graphic(900 - 30, 30, 25)
        self.create_static_node_graphic(900 - 30, 600 - 30, 25)
        self.create_static_node_graphic(900 - 30, 300 - 30, 25)
        self.create_static_node_graphic(900 - 30, 900 - 30, 25)
        self.create_static_node_graphic(600 - 30, 900 - 30, 25)
        self.create_static_node_graphic(300 - 30, 900 - 30, 25)
        self.create_static_node_graphic(30, 900 - 30, 25)
        self.create_static_node_graphic(30, 600 - 30, 25)
        self.create_static_node_graphic(30, 300 - 30, 25)

    def create_static_node_graphic(self, pos_x, pos_y, size):
        """
        Create the graphic for the static node.
        """
        self.create_polygon(
            pos_x + size,
            pos_y,  # Vertex A
            pos_x + (size / 2),
            pos_y + math.sqrt(3) * size / 2,  # Vertex B
            pos_x - (size / 2),
            pos_y + math.sqrt(3) * size / 2,  # Vertex B
            pos_x - size,
            pos_y,  # Vertex D
            pos_x - (size / 2),
            pos_y - math.sqrt(3) * size / 2,  # Vertex E
            pos_x + (size / 2),
            pos_y - math.sqrt(3) * size / 2,  # Vertex F
            fill=SECONDARY_TEXT,
        )


class DataDisplayContainer(tk.Canvas):
    def __init__(self, master):
        super().__init__(master, bg=CARD, height=900, width=450, highlightthickness=0)
        self._master = master


class DataDisplay(object):
    def __init__(self, canvas, master=None):
        self.canvas = canvas
        # Create Labels
        self.canvas.create_text(
            200,
            30,
            text="Multilateration Position:",
            font="Montserrat, 12",
            fill="#E2703A",
            anchor="e",
        )
        self.canvas.create_text(
            200,
            50,
            text="Kalman Position:",
            font="Montserrat, 12",
            fill="#9C3D54",
            anchor="e",
        )
        for idx in range(0, 12):
            self.canvas.create_text(
                200,
                80 + idx * 20,
                text="RSSI Node {}:".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="e",
            )
        for idx in range(0, 12):
            self.canvas.create_text(
                200,
                350 + idx * 20,
                text="Distance Node {}:".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="e",
            )
        for idx in range(0, 4):
            self.canvas.create_text(
                200,
                610 + idx * 20,
                text="Ultrasonic Distance {}:".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="e",
            )
        self.canvas.create_text(
            200,
            710,
            text="Accelerometer:",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            200, 740, text="Gyro:", font="Montserrat, 12", fill=PRIMARY_TEXT, anchor="e"
        )
        self.canvas.create_text(
            200,
            770,
            text="Magnetometer:",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            200,
            800,
            text="Timestamp:",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            200,
            830,
            text="Delay Time:",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )

        # Create values
        self.multilat_pos = self.canvas.create_text(
            250, 30, text="NO DATA", font="Montserrat, 12", fill="#E2703A", anchor="w"
        )
        self.kalman_pos = self.canvas.create_text(
            250, 50, text="NO DATA", font="Montserrat, 12", fill="#9C3D54", anchor="w"
        )
        self.rssi = [0] * 12
        for idx in range(0, 12):
            self.rssi[idx] = self.canvas.create_text(
                250,
                80 + idx * 20,
                text="NO DATA".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="w",
            )
        self.distance = [0] * 12
        for idx in range(0, 12):
            self.distance[idx] = self.canvas.create_text(
                250,
                350 + idx * 20,
                text="NO DATA".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="w",
            )
        self.ultra = [0] * 4
        for idx in range(0, 4):
            self.ultra[idx] = self.canvas.create_text(
                250,
                610 + idx * 20,
                text="NO DATA".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="w",
            )
        self.accel = self.canvas.create_text(
            250,
            710,
            text="NO DATA",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="w",
        )
        self.gyro = self.canvas.create_text(
            250,
            740,
            text="NO DATA",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="w",
        )
        self.mag = self.canvas.create_text(
            250,
            770,
            text="NO DATA",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="w",
        )
        self.time = self.canvas.create_text(
            250,
            800,
            text="NO DATA",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="w",
        )
        self.delay = self.canvas.create_text(
            250,
            830,
            text="NO DATA",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="w",
        )

    def update_data(self, data):
        """
        Redraw the position of the mobile node.
        """
        self.canvas.itemconfig(
            self.multilat_pos, text=f"({data.multilat_pos[0]}, {data.multilat_pos[1]})"
        )
        self.canvas.itemconfig(
            self.kalman_pos, text=f"({data.kalman_pos[0]}, {data.kalman_pos[1]})"
        )
        for idx in range(0, 12):
            self.canvas.itemconfig(self.rssi[idx], text=f"{data.node_rssi[idx]}")
        for idx in range(0, 12):
            self.canvas.itemconfig(
                self.distance[idx], text=f"{data.node_distance[idx]}"
            )
        for idx in range(0, 4):
            self.canvas.itemconfig(self.ultra[idx], text=f"{data.node_ultra[idx]}")
        self.canvas.itemconfig(
            self.accel, text=f"({data.accel[0]}, {data.accel[1]}, {data.accel[2]})"
        )
        self.canvas.itemconfig(
            self.gyro, text=f"({data.gyro[0]}, {data.gyro[1]}, {data.gyro[2]})"
        )
        self.canvas.itemconfig(
            self.mag, text=f"({data.mag[0]}, {data.mag[1]}, {data.mag[2]})"
        )
        self.canvas.itemconfig(self.time, text=f"{data.timestamp}")
        self.canvas.itemconfig(self.delay, text=f"{data.delay}")


class MobileNode(object):
    """
    Class for the mobile node
    """

    def __init__(self, canvas, type, master=None):
        self.current_x = START_POS_X
        self.current_y = START_POS_Y
        self.target_x = START_POS_X
        self.target_y = START_POS_Y
        self.canvas = canvas
        if type == "multilateration":
            self.node_colour = "#E2703A"
        elif type == "kalman":
            self.node_colour = "#9C3D54"

        self.graphic = self.canvas.create_oval(
            425, 425, 475, 475, fill=self.node_colour
        )
        self.text_position = self.canvas.create_text(
            450, 500, text="(500,500)", fill=PRIMARY_TEXT
        )

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
        self.canvas.itemconfig(
            self.text_position, text="({},{})".format(self.current_x, self.current_y)
        )


# GUI Interface ===============================================================


def gui_thread(in_q):
    """
    GUI interface for the application.
    """
    root = tk.Tk()
    app = MainApplication(in_q, master=root)
    app.mainloop()
