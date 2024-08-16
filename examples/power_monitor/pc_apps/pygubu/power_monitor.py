#!/usr/bin/python3
#
# Copyright (C) 2024 Hery Dang (henrydang@mijoconnected.com)
#
# SPDX-License-Identifier: Apache-2.0
#

import os
import json
import queue
import time
import serial
import binascii
import pathlib
import tkinter as tk
from tkinter import messagebox
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
MAX_DATA_SIZE = 20000      # Maximum number of samples for zoom-out
DAC_VCC = 4.75             # DAC VCC power supply voltage
DATA_MAX_4P2 = 3622        # DATA_MAX_4P2 = 4096 * 4.2 / DAC_VCC
DATA_3P8 = 3350            # Default VBAT output = 3.8V

WAVEFORM_UPDATE_INTERVAL = 4 # In milisecon

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

# File path for settings.ini
file_path = "settings.ini"

# Default settings
default_settings = {
    "serial_port_cmd":  "COM13",
    "serial_port_data": "COM14",
    "baudrate":         "10000000",
    "conversion_times": "280uS",
    "average_num":      "AVG_NUM_1",
    "adc_range":        "RANGE_0",
    "vbat":             "1927",
    "vbat_ena":         "False"
}

# API to read and write specific key values
class SettingsManager:
    def __init__(self, file_path):
        self.file_path = file_path

    def read_value(self, key):
        with open(self.file_path, "r") as file:
            settings = json.load(file)
        return settings.get(key)

    def write_value(self, key, value):
        with open(self.file_path, "r") as file:
            settings = json.load(file)
        settings[key] = value
        with open(self.file_path, "w") as file:
            json.dump(settings, file, indent=4)

class Power_Monitor:
    def __init__(self, master=None, on_first_object_cb=None):
        # Initialize the stack to keep track of xlim history
        self.xlim_stack = []
        # Initialize other properties
        self.original_xlim = None

        self.serial_port_cmd = None
        self.serial_port_data = None
        self.is_receiving = False
        self.is_measuring = False
        self.receive_thread = None
        self.current_data = np.array([])  # Store received current data here
        self.voltage_data = np.array([])  # Store received voltage data here
        self.data_queue_voltage = queue.Queue()
        self.data_queue_current = queue.Queue()

        self.builder = pygubu.Builder(
            on_first_object=on_first_object_cb)
        self.builder.add_resource_paths(RESOURCE_PATHS)
        self.builder.add_from_file(PROJECT_UI)
        # Main widget
        self.mainwindow: tk.Toplevel = self.builder.get_object(
            "toplevel", master)
        self.builder.connect_callbacks(self)

        # Collect the GUI objects here
        self.output_text = self.builder.get_object('text_status', master)
        self.canvas_current = self.builder.get_object('canvas_current', master)
        self.canvas_voltage = self.builder.get_object('canvas_voltage', master)
        self.avg_current_entry = self.builder.get_object('entry_average', master)
        self.marker1_text = self.builder.get_object('entry_marker1_value', master)
        self.marker2_text = self.builder.get_object('entry_marker2_value', master)
        self.entry_port_cmd = self.builder.get_object('entry_port_cmd', master)
        self.entry_port_data = self.builder.get_object('entry_port_data', master)
        self.baudrate_entry = self.builder.get_object('entry_baudrate', master)
        self.optionmenu_convtime = self.builder.get_object('optionmenu_convtime', master)
        self.optionmenu_avgnum = self.builder.get_object('optionmenu_avgnum', master)
        self.optionmenu_adcrange = self.builder.get_object('optionmenu_adcrange', master)
        self.input_entry = self.builder.get_object('entry_cmd', master)
        self.scale_vbat = self.builder.get_object('scale_voltage_out', master)
        self.entry_vbat = self.builder.get_object('entry_vbat_value', master)
        self.checkbt_vbat_ena = self.builder.get_object('checkbutton_vbat_enable', master)
        self.entry_min = self.builder.get_object('entry_min', master)
        self.entry_max = self.builder.get_object('entry_max', master)

        # Check if the settings file exists, if not, create it with default values
        if not os.path.exists(file_path):
            with open(file_path, "w") as file:
                json.dump(default_settings, file, indent=4)

        # Get port and baudrate from settings.ini
        self.settings_manager = SettingsManager(file_path)

        # Load the settings
        self.load_settings()

        # Link scrollbars
        self.vscroll = self.builder.get_object('scrollbar_vertical', master)
        self.hscroll = self.builder.get_object('scrollbar_horizontal', master)
        self.output_text.config(yscrollcommand=self.vscroll.set, xscrollcommand=self.hscroll.set)
        self.vscroll.config(command=self.output_text.yview)
        self.hscroll.config(command=self.output_text.xview)

        # Matplotlib figure for plotting current waveform
        self.figure1 = plt.Figure(figsize=(20, 3), dpi=70)
        self.ax1 = self.figure1.add_subplot(111)
        self.ax1.set_title("Current Waveform (mA)")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("Current (mA)")
        self.canvas1 = FigureCanvasTkAgg(self.figure1, master=self.canvas_current)
        self.canvas1.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Matplotlib figure for plotting volatge waveform
        self.figure2 = plt.Figure(figsize=(20, 2), dpi=70)
        self.ax2 = self.figure2.add_subplot(111)
        self.ax2.set_title("Volatge Waveform (V)")
        self.ax2.set_xlabel("Sample")
        self.ax2.set_ylabel("Volatage (V)")
        self.canvas2 = FigureCanvasTkAgg(self.figure2, master=self.canvas_voltage)
        self.canvas2.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Initialize marker positions
        self.marker1_pos = 200
        self.marker2_pos = 400
        # Add default markers
        self.marker_line1 = self.ax1.axvline(self.marker1_pos, color='red', linestyle='--')
        self.marker_line2 = self.ax1.axvline(self.marker2_pos, color='blue', linestyle='--')
        # Initialize dragging_marker
        self.dragging_marker = None
        # Connect event handlers for dragging markers
        self.canvas1.mpl_connect('button_press_event', self.on_press)
        self.canvas1.mpl_connect('button_release_event', self.on_release)
        self.canvas1.mpl_connect('motion_notify_event', self.on_motion)
        self.canvas1.mpl_connect('scroll_event', self.on_scroll)
        # Update the waveform and markers
        self.update_current_waveform(self.current_data)
        # Schedule voltage/current update waveform
        self.mainwindow.after(WAVEFORM_UPDATE_INTERVAL, self.update_waveform)
        # Bind the close event to the custom close method
        self.mainwindow.protocol("WM_DELETE_WINDOW", self.close)

    def load_settings(self):
        # Connection settings
        self.entry_port_cmd.config(state=tk.NORMAL)
        self.entry_port_cmd.delete(0, tk.END)
        self.entry_port_cmd.insert(0, self.settings_manager.read_value("serial_port_cmd"))
        self.entry_port_data.config(state=tk.NORMAL)
        self.entry_port_data.delete(0, tk.END)
        self.entry_port_data.insert(0, self.settings_manager.read_value("serial_port_data"))
        self.baudrate_entry.config(state=tk.NORMAL)
        self.baudrate_entry.delete(0, tk.END)
        self.baudrate_entry.insert(0, self.settings_manager.read_value("baudrate"))

        # Load conversion time drop-down list
        self.selected_convtime_key = tk.StringVar(None)
        self.selected_convtime_key.set(self.settings_manager.read_value("conversion_times"))
        self.update_optionmenu_convtime_items()
        self.optionmenu_convtime['textvariable'] = self.selected_convtime_key

        # Load average num drop-down list
        self.selected_avgnum_key = tk.StringVar(None)
        self.selected_avgnum_key.set(self.settings_manager.read_value("average_num"))
        self.update_optionmenu_avgnum_items()
        self.optionmenu_avgnum['textvariable'] = self.selected_avgnum_key

        # Load adc range drop-down list
        self.selected_adcrange_key = tk.StringVar(None)
        self.selected_adcrange_key.set(self.settings_manager.read_value("adc_range"))
        self.update_optionmenu_adcrange_items()
        self.optionmenu_adcrange['textvariable'] = self.selected_adcrange_key

        # Load VBAT settings
        self.scale_vbat.configure(from_=0, to=DATA_MAX_4P2)
        self.scale_vbat.set(int(self.settings_manager.read_value("vbat")))
        int_value = int(float(self.scale_vbat.get()))
        vbat_voltage = (DAC_VCC * int_value) / 4096
        self.entry_vbat.config(state=tk.NORMAL)
        self.entry_vbat.delete(0, tk.END)
        self.entry_vbat.insert(0, f"{vbat_voltage:.2f}")

        # Load VBAT output enable
        self.check_var = tk.BooleanVar()
        if self.settings_manager.read_value("vbat_ena") == "True":
            self.check_var.set(True)
        else:
            self.check_var.set(False)
        self.checkbt_vbat_ena.config(variable=self.check_var, onvalue=True, offvalue=False)
        self.check_var.trace_add("write", self.on_change_vbat_enable)

    def store_settings(self):
        self.settings_manager.write_value("serial_port_cmd", self.entry_port_cmd.get())
        self.settings_manager.write_value("serial_port_data", self.entry_port_data.get())
        self.settings_manager.write_value("baudrate", self.baudrate_entry.get())
        self.settings_manager.write_value("conversion_times", self.selected_convtime_key.get())
        self.settings_manager.write_value("average_num", self.selected_avgnum_key.get())
        self.settings_manager.write_value("adc_range", self.selected_adcrange_key.get())
        self.settings_manager.write_value("vbat", str(int(float(self.scale_vbat.get()))))
        if self.check_var.get():
            self.settings_manager.write_value("vbat_ena", "True")
        else:
            self.settings_manager.write_value("vbat_ena", "False")

    def update_optionmenu_convtime_items(self):
        menu = self.optionmenu_convtime['menu']
        menu.delete(0, 'end')  # Clear existing items
        for key in conversion_times.keys():
            menu.add_command(label=key, command=tk._setit(self.selected_convtime_key, key))

    def update_optionmenu_avgnum_items(self):
        menu = self.optionmenu_avgnum['menu']
        menu.delete(0, 'end')  # Clear existing items
        for key in average_num.keys():
            menu.add_command(label=key, command=tk._setit(self.selected_avgnum_key, key))

    def update_optionmenu_adcrange_items(self):
        menu = self.optionmenu_adcrange['menu']
        menu.delete(0, 'end')  # Clear existing items
        for key in adc_range.keys():
            menu.add_command(label=key, command=tk._setit(self.selected_adcrange_key, key))

    def update_marker_values(self):
        # Ensure marker positions are within the bounds of current_data
        marker1_index = int(self.marker1_pos)
        marker2_index = int(self.marker2_pos)

        # Get the current values at marker positions
        marker1_value = self.current_data[marker1_index] if 0 <= marker1_index < len(self.current_data) else 'N/A'
        marker2_value = self.current_data[marker2_index] if 0 <= marker2_index < len(self.current_data) else 'N/A'

        # Update text boxes
        self.marker1_text.delete(0, tk.END)
        self.marker2_text.delete(0, tk.END)

        # Helper function to format values
        def format_value(value):
            try:
                return f"{float(value):.2f}"  # Convert to float and format
            except ValueError:
                return str(value)  # Return the value as is if it can't be converted

        self.marker1_text.insert(0, format_value(marker1_value))
        self.marker2_text.insert(0, format_value(marker2_value))

    def on_change_vbat_enable(self, *args):
        # Prepare command "Battery simulator volatge output enable/disable"
        cmd = bytearray()
        if self.check_var.get():
            #print("Checkbutton is checked")
            cmd.extend([0x06, 0x01, 0x00, 0x00])
        else:
            cmd.extend([0x06, 0x00, 0x00, 0x00])
            #print("Checkbutton is unchecked")

        if not self.serial_port_cmd or not self.serial_port_cmd.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return
        try:
            # Write adc config param command
            self.serial_port_cmd.write(cmd)
            response = self.serial_port_cmd.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return
        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.output_text.see(tk.END)

    def on_scale_change(self, value):
        int_value = int(float(value))
        vbat_voltage = (DAC_VCC * int_value) / 4096
        self.entry_vbat.config(state=tk.NORMAL)
        self.entry_vbat.delete(0, tk.END)
        self.entry_vbat.insert(0, f"{vbat_voltage:.2f}")
        # print(f"Scale value changed to: {int_value}")

    def on_set_vbat_value(self):
        # Get VBAT setting value
        int_value = int(float(self.scale_vbat.get()))
        low_byte = int_value & 0xFF
        high_byte = (int_value >> 8) & 0xFF

        # Prepare command "set battery simulator volatge"
        cmd = bytearray()
        cmd.extend([0x05, high_byte, low_byte, 0x00])
        self.output_text.insert(tk.END, f"Command: {cmd}\n")

        if not self.serial_port_cmd or not self.serial_port_cmd.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return
        try:
            # Write adc config param command
            self.serial_port_cmd.write(cmd)
            response = self.serial_port_cmd.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return
        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.output_text.see(tk.END)

    def on_scroll(self, event):
        if self.is_measuring == True:
            return

        if event.inaxes != self.ax1:
            return

        zoom_factor = 1.1
        if event.button == 'up':
            zoom_factor = 1 / zoom_factor
            self.xlim_stack.append(self.ax1.get_xlim())
        elif event.button == 'down':
            if self.xlim_stack:
                prev_xlim = self.xlim_stack.pop()
                self.ax1.set_xlim(prev_xlim)
            else:
                self.ax1.set_xlim(self.original_xlim)

        x_center = (self.marker1_pos + self.marker2_pos) / 2
        x_range = self.ax1.get_xlim()[1] - self.ax1.get_xlim()[0]
        new_x_range = x_range * zoom_factor

        if event.button == 'up':
            self.ax1.set_xlim([x_center - new_x_range / 2, x_center + new_x_range / 2])
        elif event.button == 'down':
            new_xlim = [x_center - new_x_range / 2, x_center + new_x_range / 2]
            # Ensure limits do not exceed original limits on zoom out
            self.ax1.set_xlim([
                max(new_xlim[0], self.original_xlim[0]),
                min(new_xlim[1], self.original_xlim[1])
            ])

        self.canvas1.draw()

    def on_press(self, event):
        if self.is_measuring == True:
            return

        if event.inaxes != self.ax1:
            return

        # Check if the click is close to a marker
        x_center = (self.marker1_pos + self.marker2_pos) / 2

        if(event.xdata <= self.marker1_pos) and (abs(event.xdata - self.marker1_pos) < x_center):
            self.dragging_marker = 'marker1'
        elif (event.xdata >= self.marker2_pos) and (abs(event.xdata - self.marker2_pos) < x_center):
            self.dragging_marker = 'marker2'

    def on_release(self, event):
        if self.is_measuring == True:
            return

        self.dragging_marker = None

    def on_motion(self, event):
        if self.is_measuring == True:
            return

        if not hasattr(self, 'dragging_marker'):
            return

        if event.inaxes != self.ax1:
            return

        # Update marker position based on mouse movement
        if self.dragging_marker == 'marker1':
            self.marker1_pos = min(max(event.xdata, 0), self.marker2_pos)
            self.marker_line1.set_xdata([self.marker1_pos, self.marker1_pos])
        elif self.dragging_marker == 'marker2':
            self.marker2_pos = max(min(event.xdata, len(self.current_data)), self.marker1_pos)
            self.marker_line2.set_xdata([self.marker2_pos, self.marker2_pos])

        # Update marker values in text boxes based on current data
        self.update_marker_values()

        # Recalculate the average current and update the display
        self.calculate_and_update_average()

        # Redraw the canvas
        self.canvas1.draw()

    def update_current_waveform(self, current_data):
        # Clear the axis without removing the markers
        self.ax1.clear()

        # Plot only the last MAX_DATA_SIZE samples if the data size exceeds it
        if len(current_data) > MAX_DATA_SIZE:
            self.ax1.plot(current_data[-MAX_DATA_SIZE:], color = "green")
            self.original_xlim = [0, MAX_DATA_SIZE]
        else:
            self.ax1.plot(current_data, color = "green")
            self.original_xlim = [0, len(current_data)]

        # Re-add the markers after plotting the waveform
        self.marker_line1 = self.ax1.axvline(self.marker1_pos, color='red', linestyle='--')
        self.marker_line2 = self.ax1.axvline(self.marker2_pos, color='blue', linestyle='--')

        # Set axis labels and title
        self.ax1.set_title("Current Waveform (mA)")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("Current (mA)")

        # Update average current display
        self.calculate_and_update_average()

        # Check and set initial x-limits
        current_xlim = self.ax1.get_xlim()
        if current_xlim[0] == current_xlim[1]:  # Check if x-limits are identical
            # Increase the range slightly to avoid identical limits
            delta = 1
            new_xlim = [current_xlim[0] - delta, current_xlim[1] + delta]
        else:
            new_xlim = self.original_xlim

        # Ensure new_xlim is valid and has distinct bounds
        if new_xlim[0] == new_xlim[1]:
            new_xlim[0] -= 1
            new_xlim[1] += 1

        self.ax1.set_xlim(new_xlim)
        self.canvas1.draw()

    def update_voltage_waveform(self, voltage_data):
        self.ax2.clear()
        self.ax2.plot(voltage_data, color = "orange")
        self.ax1.set_title("Volatge Waveform (V)")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("Voltage (mA)")
        self.canvas2.draw()

    def calculate_and_update_average(self):
        if not hasattr(self, 'avg_current_entry'):
            return  # Exit if avg_current_entry is not available

        x_min, x_max = int(self.marker1_pos), int(self.marker2_pos)

        if x_min < 0:
            x_min = 0
        if x_max > len(self.current_data):
            x_max = len(self.current_data)

        selected_data = self.current_data[x_min:x_max]

        # Remove invalid values
        selected_data = selected_data[np.isfinite(selected_data)]

        if len(selected_data) > 0:
            avg_current = np.mean(selected_data)
            min_current = np.min(selected_data)
            max_current = np.max(selected_data)
        else:
            avg_current = 0  # Or another appropriate value
            min_current = 0
            max_current = 0

        self.avg_current_entry.config(state=tk.NORMAL)
        self.avg_current_entry.delete(0, tk.END)
        self.avg_current_entry.insert(0, f"{avg_current:.2f}")
        self.avg_current_entry.config(state="readonly")

        self.entry_min.config(state=tk.NORMAL)
        self.entry_min.delete(0, tk.END)
        self.entry_min.insert(0, f"{min_current:.2f}")
        self.entry_min.config(state="readonly")

        self.entry_max.config(state=tk.NORMAL)
        self.entry_max.delete(0, tk.END)
        self.entry_max.insert(0, f"{max_current:.2f}")
        self.entry_max.config(state="readonly")

    def execute_stop_measuring(self):
        cmd = bytearray()

        if not self.serial_port_cmd or not self.serial_port_cmd.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        try:
            # Run command start measuring
            cmd = bytearray([0x08, 0x00, 0x00, 0x00])
            self.serial_port_cmd.write(cmd)
            response = self.serial_port_cmd.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            # We don't expect response OK after stop measure command
            # if response[0] != cmd[0] or response[1] != 0x01:
                # messagebox.showerror("Error", "Device respone error")
                # return

        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.output_text.see(tk.END)

        # Update the display to show markers even without current data
        self.is_measuring = False
        self.update_current_waveform(self.current_data)

    def execute_start_measuring(self):
        cmd = bytearray()

        if not self.serial_port_cmd or not self.serial_port_cmd.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        try:
            # Run command start measuring
            cmd = bytearray([0x07, 0x00, 0x00, 0x00])
            self.serial_port_cmd.write(cmd)
            response = self.serial_port_cmd.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return

        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.is_measuring = True
        self.output_text.see(tk.END)

    def execute_adc_configuration(self):
        cmd = bytearray()
        selected_conv_time = self.selected_convtime_key.get()
        hex_value = conversion_times[selected_conv_time]
        cmd.extend([0x02, 0x00, 0x00, 0x00]) # Command write config
        cmd.append(hex_value) # added cnv_time
        self.output_text.insert(tk.END, f"Selected Conversion Time: {selected_conv_time} (0x{hex_value:X})\n")

        selected_avg_num = self.selected_avgnum_key.get()
        hex_value = average_num[selected_avg_num]
        cmd.append(hex_value) # added avg_num
        self.output_text.insert(tk.END, f"Selected Average Num: {selected_avg_num} (0x{hex_value:X})\n")

        selected_adc_range = self.selected_adcrange_key.get()
        hex_value = adc_range[selected_adc_range]
        cmd.append(hex_value) # added adc_range
        cmd.append(0x01)      # added avg_alert
        self.output_text.insert(tk.END, f"Selected Adc Range: {selected_adc_range} (0x{hex_value:X})\n")
        self.output_text.insert(tk.END, f"Command: {cmd}\n")

        if not self.serial_port_cmd or not self.serial_port_cmd.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        try:
            # Write adc config param command
            self.serial_port_cmd.write(cmd)
            response = self.serial_port_cmd.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return

            # Run command configure INA229
            cmd = bytearray([0x04, 0x00, 0x00, 0x00])
            self.serial_port_cmd.write(cmd)
            response = self.serial_port_cmd.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return

        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.output_text.see(tk.END)

    def connect(self):
        port_cmd = self.entry_port_cmd.get()
        port_data = self.entry_port_data.get()
        baudrate = self.baudrate_entry.get()
        try:
            self.serial_port_cmd = serial.Serial(port_cmd, baudrate=int(baudrate), timeout=1)
            self.serial_port_data = serial.Serial(port_data, baudrate=int(baudrate), timeout=1)
            # messagebox.showinfo("Connection", f"Connected to {port_cmd} at {baudrate} baud")
            self.is_receiving = True
            self.receive_thread = threading.Thread(target=self.receive_data)
            self.receive_thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.is_receiving = False
        if self.receive_thread:
            self.receive_thread.join()
        if self.serial_port_cmd and self.serial_port_cmd.is_open:
            self.serial_port_cmd.flushInput()
            self.serial_port_cmd.flushOutput()
            self.serial_port_cmd.close()
            # messagebox.showinfo("Disconnection", "Disconnected from the UART cmd port")
        if self.serial_port_data and self.serial_port_data.is_open:
            self.serial_port_data.flushInput()
            self.serial_port_data.flushOutput()
            self.serial_port_data.close()
            # messagebox.showinfo("Disconnection", "Disconnected from the UART data port")

    def clear_output(self):
        self.output_text.delete('1.0', tk.END)

    def close(self):
        self.store_settings()
        self.disconnect()
        self.mainwindow.destroy()

    def send_data(self):
        if not self.serial_port_cmd or not self.serial_port_cmd.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        hex_input = self.input_entry.get().strip()

        try:
            # Convert space-separated hex input to bytes
            data = binascii.unhexlify(hex_input.replace(" ", ""))
            self.serial_port_cmd.write(data)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def receive_data(self):
        while self.is_receiving:
            try:
                if self.serial_port_data.in_waiting >= (4 + 4 + 4 * DATA_RPT_SAMPLE_SIZE * 2):
                    data = self.serial_port_data.read(4 + 4 + 4 * DATA_RPT_SAMPLE_SIZE * 2)

                    # Unpack the received data
                    sign, package_id = struct.unpack('<II', data[:8])

                    # Check the signature
                    if sign == SIGNATURE:
                        voltage_data = struct.unpack('<' + 'i' * DATA_RPT_SAMPLE_SIZE, data[8:8 + 4 * DATA_RPT_SAMPLE_SIZE])
                        current_data = struct.unpack('<' + 'i' * DATA_RPT_SAMPLE_SIZE, data[8 + 4 * DATA_RPT_SAMPLE_SIZE:])
                        #print(f"Voltage: {voltage}\nCurrent: {current}")  # Debug output
                        self.data_queue_voltage.put(voltage_data)
                        self.data_queue_current.put(current_data)
            except Exception as e:
                self.is_receiving = False
                break

            time.sleep(0.1)

    def update_waveform(self):
        # Voltage data dequeue processing
        try:
            while True:
                voltage_data = self.data_queue_voltage.get_nowait()
                self.voltage_data = np.append(self.voltage_data, voltage_data)
                if len(self.voltage_data) > MAX_DATA_SIZE:
                    self.voltage_data = self.voltage_data[-MAX_DATA_SIZE:]
                self.update_voltage_waveform(self.voltage_data)
        except queue.Empty:
            pass
        # Current data dequeue processing
        try:
            while True:
                current_data = self.data_queue_current.get_nowait()
                self.current_data = np.append(self.current_data, current_data)
                if len(self.current_data) > MAX_DATA_SIZE:
                    self.current_data = self.current_data[-MAX_DATA_SIZE:]
                self.update_current_waveform(self.current_data)
        except queue.Empty:
            pass

        self.mainwindow.after(WAVEFORM_UPDATE_INTERVAL, self.update_waveform)

    def run(self):
        self.mainwindow.mainloop()

if __name__ == "__main__":
    app = Power_Monitor()
    app.run()
