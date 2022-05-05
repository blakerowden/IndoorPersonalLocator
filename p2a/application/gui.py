"""
Prac 2a - Desktop Application
.py file 4/7 - GUI
CSSE4011 - Advanced Embedded Systems
Semester 1, 2022
"""

__author__ = "B.Rowden"

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
GRID_LENGTH_PX = 900
HALF_GRID_PX = GRID_LENGTH_PX / 2
THIRD_GRID_PX = GRID_LENGTH_PX / 3
TWO_THIRD_GRID_PX = THIRD_GRID_PX * 2
PX_OFFSET = 30
CM_TO_PX = GRID_LENGTH_PX / 400
PX_TO_CM = 400 / GRID_LENGTH_PX
TOTAL_GRID_LINES = 6
LINE_SPACE = int(GRID_LENGTH_PX / TOTAL_GRID_LINES)

TITLE = "Prac 2 - Find my Thingy:52"

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

        title_active = False
        if title_active:
            title = tk.Label(
                self._master,
                text=TITLE,
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
        self._fusion_node = MobileNode(self._grid, "sensor_fusion")
        self._ml_node = MobileNode(self._grid, "ml")

        self._data_container = DataDisplayContainer(self._master)
        self._data_container.place(relx=0.35, rely=0.5, anchor=tk.E)
        self._data = DataDisplay(self._data_container)

        self._master.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.refresh_application()

    def on_closing(self):
        """Handle the closing of the window."""
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

                self._multilateration_node.target_x = data.multilat_pos[0] * CM_TO_PX
                self._multilateration_node.target_y = data.multilat_pos[1] * CM_TO_PX

                self._fusion_node.target_x = data.k_multilat_pos[0] * CM_TO_PX
                self._fusion_node.target_y = data.k_multilat_pos[1] * CM_TO_PX

                self._ml_node.target_x = data.k_ml_pos[0] * CM_TO_PX
                self._ml_node.target_y = data.k_ml_pos[1] * CM_TO_PX

                self._data.update_data(data)

            if self._stop:
                break
            # Update the GUI

            self._multilateration_node.redraw_position()
            self._fusion_node.redraw_position()
            self._ml_node.redraw_position()
            self._master.update()


class Grid(tk.Canvas):
    """
    Class for the grid.
    """

    def __init__(self, master):
        super().__init__(
            master,
            bg=CARD,
            height=GRID_LENGTH_PX,
            width=GRID_LENGTH_PX,
            highlightthickness=0,
        )
        self._master = master

        # create a grid
        for i in range(0, GRID_LENGTH_PX, LINE_SPACE):
            self.create_line(i, 0, i, GRID_LENGTH_PX, fill=SECONDARY_TEXT, width=2)
        for i in range(0, GRID_LENGTH_PX, LINE_SPACE):
            self.create_line(0, i, GRID_LENGTH_PX, i, fill=SECONDARY_TEXT, width=2)
        self.create_line(0, 0, GRID_LENGTH_PX, 0, fill=SECONDARY_TEXT, width=6)
        self.create_line(
            GRID_LENGTH_PX,
            0,
            GRID_LENGTH_PX,
            GRID_LENGTH_PX,
            fill=SECONDARY_TEXT,
            width=6,
        )
        self.create_line(
            GRID_LENGTH_PX,
            GRID_LENGTH_PX,
            0,
            GRID_LENGTH_PX,
            fill=SECONDARY_TEXT,
            width=6,
        )
        self.create_line(0, GRID_LENGTH_PX, 0, 0, fill=SECONDARY_TEXT, width=6)

        self.create_static_node_graphic(PX_OFFSET, PX_OFFSET, 25, "A")
        self.create_static_node_graphic(THIRD_GRID_PX - PX_OFFSET, PX_OFFSET, 25, "B*")
        self.create_static_node_graphic(
            TWO_THIRD_GRID_PX - PX_OFFSET, PX_OFFSET, 25, "C"
        )
        self.create_static_node_graphic(GRID_LENGTH_PX - PX_OFFSET, PX_OFFSET, 25, "D")
        self.create_static_node_graphic(
            GRID_LENGTH_PX - PX_OFFSET, THIRD_GRID_PX - PX_OFFSET, 25, "E*"
        )
        self.create_static_node_graphic(
            GRID_LENGTH_PX - PX_OFFSET, TWO_THIRD_GRID_PX - PX_OFFSET, 25, "F"
        )
        self.create_static_node_graphic(
            GRID_LENGTH_PX - PX_OFFSET, GRID_LENGTH_PX - PX_OFFSET, 25, "G"
        )
        self.create_static_node_graphic(
            TWO_THIRD_GRID_PX - PX_OFFSET, GRID_LENGTH_PX - PX_OFFSET, 25, "H*"
        )
        self.create_static_node_graphic(
            THIRD_GRID_PX - PX_OFFSET, GRID_LENGTH_PX - PX_OFFSET, 25, "I"
        )
        self.create_static_node_graphic(PX_OFFSET, GRID_LENGTH_PX - PX_OFFSET, 25, "J")
        self.create_static_node_graphic(
            PX_OFFSET, TWO_THIRD_GRID_PX - PX_OFFSET, 25, "K*"
        )
        self.create_static_node_graphic(PX_OFFSET, THIRD_GRID_PX - PX_OFFSET, 25, "L")

    def create_static_node_graphic(self, pos_x, pos_y, size, letter):
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
        self.create_text(
            pos_x, pos_y, text=letter, fill=BACKGROUND, font="Montserrat, 17"
        )


class DataDisplayContainer(tk.Canvas):
    def __init__(self, master):
        super().__init__(
            master,
            bg=CARD,
            height=GRID_LENGTH_PX,
            width=HALF_GRID_PX,
            highlightthickness=0,
        )
        self._master = master


class DataDisplay(object):
    def __init__(self, canvas, master=None):
        self.canvas = canvas
        # Create Labels
        data_label_offset = 220
        self.canvas.create_text(
            data_label_offset,
            PX_OFFSET,
            text="Multilat Co-ords (m):",
            font="Montserrat, 12",
            fill="#E2703A",
            anchor="e",
        )
        self.canvas.create_text(
            data_label_offset,
            50,
            text="Data Fusion Co-ords (m):",
            font="Montserrat, 12",
            fill="#3E7CB1",
            anchor="e",
        )
        self.canvas.create_text(
            data_label_offset,
            70,
            text="ML Co-ords (m):",
            font="Montserrat, 12",
            fill="#B9FFB7",
            anchor="e",
        )
        for idx in range(0, 12):
            self.canvas.create_text(
                data_label_offset,
                100 + idx * 20,
                text="Node {} RSSI (dB):".format(chr(65 + idx)),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="e",
            )
        for idx in range(0, 12):
            self.canvas.create_text(
                data_label_offset,
                350 + idx * 20,
                text="Node {} Distance (cm):".format(chr(65 + idx)),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="e",
            )
        for idx in range(0, 4):
            self.canvas.create_text(
                data_label_offset,
                610 + idx * 20,
                text="Ultra {} Distance (cm):".format(idx),
                font="Montserrat, 12",
                fill=PRIMARY_TEXT,
                anchor="e",
            )
        self.canvas.create_text(
            data_label_offset,
            710,
            text="Accelerometer (m/s):",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            data_label_offset,
            740,
            text="Gyro (rad):",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            data_label_offset,
            770,
            text="Magnetometer:",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            data_label_offset,
            800,
            text="Timestamp:",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )
        self.canvas.create_text(
            data_label_offset,
            830,
            text="Delay Time (ms):",
            font="Montserrat, 12",
            fill=PRIMARY_TEXT,
            anchor="e",
        )

        # Create values
        self.multilat_pos = self.canvas.create_text(
            250,
            PX_OFFSET,
            text="NO DATA",
            font="Montserrat, 12",
            fill="#E2703A",
            anchor="w",
        )
        self.data_fusion_pos = self.canvas.create_text(
            250, 50, text="NO DATA", font="Montserrat, 12", fill="#3E7CB1", anchor="w"
        )
        self.ml_pos = self.canvas.create_text(
            250, 70, text="NO DATA", font="Montserrat, 12", fill="#B9FFB7", anchor="w"
        )
        self.rssi = [0] * 12
        for idx in range(0, 12):
            self.rssi[idx] = self.canvas.create_text(
                250,
                100 + idx * 20,
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
        try:
            self.canvas.itemconfig(
                self.multilat_pos,
                text=f"({math.ceil(data.multilat_pos[0])/100.0}, {math.ceil(data.multilat_pos[1])/100.0})",
            )
        except:
            return
        self.canvas.itemconfig(
            self.data_fusion_pos,
            text=f"({math.ceil(data.k_multilat_pos[0])/100.0}, {math.ceil(data.k_multilat_pos[1])/100.0})",
        )
        self.canvas.itemconfig(
            self.ml_pos,
            text=f"({math.ceil(data.k_ml_pos[0])/100.0}, {math.ceil(data.k_ml_pos[1])/100.0})",
        )
        for idx in range(0, 12):
            self.canvas.itemconfig(self.rssi[idx], text=f"{data.node_rssi[idx]}")
        for idx in range(0, 12):
            self.canvas.itemconfig(
                self.distance[idx], text=f"{math.ceil(data.node_distance[idx])}"
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
        self.canvas.itemconfig(self.delay, text=f"{data.rssi_delay}")


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
            self.speed = 1.2
        elif type == "sensor_fusion":
            self.speed = 0.2
            self.node_colour = "#3E7CB1"
        elif type == "ml":
            self.speed = 0.5
            self.node_colour = "#B9FFB7"

        self.graphic = self.canvas.create_oval(
            425, 425, 475, 475, fill=self.node_colour
        )
        self.text_coords = self.canvas.create_text(
            HALF_GRID_PX, 500, text="(2.0,2.0)", fill=PRIMARY_TEXT
        )

    def redraw_position(self):
        """
        Redraw the position of the mobile node.
        """

        if self.current_x < self.target_x:
            self.current_x += self.speed
            self.canvas.move(self.graphic, self.speed, 0)
            self.canvas.move(self.text_coords, self.speed, 0)
        elif self.current_x > self.target_x:
            self.current_x -= self.speed
            self.canvas.move(self.graphic, -self.speed, 0)
            self.canvas.move(self.text_coords, -self.speed, 0)
        if self.current_y < self.target_y:
            self.current_y += self.speed
            self.canvas.move(self.graphic, 0, self.speed)
            self.canvas.move(self.text_coords, 0, self.speed)
        elif self.current_y > self.target_y:
            self.current_y -= self.speed
            self.canvas.move(self.graphic, 0, -self.speed)
            self.canvas.move(self.text_coords, 0, -self.speed)
        self.canvas.itemconfig(
            self.text_coords,
            text="({},{})".format(
                math.ceil(self.current_x * PX_TO_CM) / 100.0,  # Convert to m
                math.ceil(self.current_y * PX_TO_CM) / 100.0,
            ),
        )


# GUI Interface ===============================================================


def gui_thread(in_q):
    """
    GUI interface for the application.
    """
    root = tk.Tk()
    app = MainApplication(in_q, master=root)
    app.mainloop()
