#!/usr/bin/python3
import pathlib
import tkinter as tk
import pygubu
import threading
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "power_monitor.ui"
RESOURCE_PATHS = [PROJECT_PATH]

# Constants
SIGNATURE = 0x87654321
DATA_RPT_SAMPLE_SIZE = 63  # The size of the current and voltage arrays
MAX_DATA_SIZE = 100000     # Maximum number of samples for zoom-out

conversion_times = {
    "280uS": 0x3,
    "540uS": 0x4,
    "1052uS": 0x5,
    "2074uS": 0x6,
    "4120uS": 0x7
}

average_num = {
    "AVG_NUM_1"    : 0x00,
    "AVG_NUM_4"    : 0x01,
    "AVG_NUM_16"   : 0x02,
    "AVG_NUM_64"   : 0x03,
    "AVG_NUM_128"  : 0x04,
    "AVG_NUM_256"  : 0x05,
    "AVG_NUM_512"  : 0x06,
    "AVG_NUM_1024" : 0x07
}

adc_range = {
    "RANGE_0"  : 0x00,
    "RANGE_1"  : 0x01
}

class auto_generateUI:
    def __init__(self, master=None, on_first_object_cb=None):
        # Initialize the stack to keep track of xlim history
        self.xlim_stack = []
        # Initialize other properties
        self.original_xlim = None

        self.serial_port = None
        self.is_receiving = False
        self.is_measuring = False
        self.receive_thread = None
        self.current_data = np.array([])  # Store received current data here
        self.voltage_data = np.array([])  # Store received voltage data here

        self.builder = pygubu.Builder(
            on_first_object=on_first_object_cb)
        self.builder.add_resource_paths(RESOURCE_PATHS)
        self.builder.add_from_file(PROJECT_UI)
        # Main widget
        self.mainwindow: tk.Toplevel = self.builder.get_object(
            "toplevel", master)
        self.builder.connect_callbacks(self)

        # Link scrollbars
        self.text_status = self.builder.get_object('text_status', master)
        self.vscroll = self.builder.get_object('scrollbar_vertical', master)
        self.hscroll = self.builder.get_object('scrollbar_horizontal', master)

        self.text_status.config(yscrollcommand=self.vscroll.set, xscrollcommand=self.hscroll.set)
        self.vscroll.config(command=self.text_status.yview)
        self.hscroll.config(command=self.text_status.xview)

        # Matplotlib figure for plotting current waveform
        self.figure1 = plt.Figure(figsize=(20, 3), dpi=70)
        self.ax1 = self.figure1.add_subplot(111)
        self.ax1.set_title("Current Waveform (mA)")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("Current (mA)")
        self.canvas_current = self.builder.get_object('canvas_current', master)
        self.canvas1 = FigureCanvasTkAgg(self.figure1, master=self.canvas_current)
        self.canvas1.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas1.draw()

        # Matplotlib figure for plotting volatge waveform
        self.figure2 = plt.Figure(figsize=(20, 2), dpi=70)
        self.ax2 = self.figure2.add_subplot(111)
        self.ax2.set_title("Volatge Waveform (V)")
        self.ax2.set_xlabel("Sample")
        self.ax2.set_ylabel("Volatage (V)")
        self.canvas_voltage = self.builder.get_object('canvas_voltage', master)
        self.canvas2 = FigureCanvasTkAgg(self.figure2, master=self.canvas_voltage)
        self.canvas2.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas2.draw()

    def run(self):
        self.mainwindow.mainloop()


if __name__ == "__main__":
    app = auto_generateUI()
    app.run()
