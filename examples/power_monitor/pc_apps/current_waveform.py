#######################################################################################################
# FreeBSD License
# Copyright (c) [2024], Henry Dang (henrydang@mijoconnected.com)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials
#    provided with the distribution.
# 3. Neither the name of Henry Dang nor the names of its contributors
#    may be used to endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#######################################################################################################

import time
import serial
import binascii
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# Constants
SIGNATURE = 0x87654321
DATA_RPT_SAMPLE_SIZE = 256  # The size of the current and voltage arrays
MAX_DATA_SIZE = 100000  # Maximum number of samples for zoom-out

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

class UARTApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Power Logger")
        self.root.geometry("1200x900")
        self.root.resizable(True, True)  # Allow the window to be maximized

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

        # UART Settings
        self.port_label = tk.Label(root, text="COM Port:")
        self.port_label.grid(row=0, column=0, padx=(155, 5), sticky="w")
        self.port_entry = tk.Entry(root)
        self.port_entry.insert(0, "COM22")
        self.port_entry.grid(row=0, column=1, padx=(0, 5), sticky="w")

        self.baudrate_label = tk.Label(root, text="Baud Rate:")
        self.baudrate_label.grid(row=0, column=1, padx=(160, 10), sticky="w")
        self.baudrate_entry = tk.Entry(root)
        self.baudrate_entry.insert(0, "10000000")  # Default baud rate set to 2000000
        self.baudrate_entry.grid(row=0, column=1, padx=(230, 360), sticky="w")

        self.connect_button = tk.Button(root, text="Connect", bg="blue", fg="white", command=self.connect)
        self.connect_button.grid(row=1, column=0, padx=(155, 20), sticky="w")

        self.disconnect_button = tk.Button(root, text="Disconnect", bg="blue", fg="white", command=self.disconnect)
        self.disconnect_button.grid(row=1, column=1, padx=(10, 20), sticky="w")

        # Data input/output
        self.input_label = tk.Label(root, text="Cmd (Hex):")
        self.input_label.grid(row=4, column=0, padx=(155, 10), sticky="w")
        self.input_entry = tk.Entry(root)
        self.input_entry.grid(row=4, column=1, padx=(0, 10), sticky="w")

        self.send_button = tk.Button(root, text="Send Cmd", command=self.send_data)
        self.send_button.grid(row=4, column=1, padx=(155, 10), sticky="w")

        # Create a conversion time drop-down list
        self.conv_label = tk.Label(root, text="Conv Time:")
        self.conv_label.grid(row=5, column=0, padx=(155, 5), sticky="w")
        self.selected_conversion_time = tk.StringVar()
        self.selected_conversion_time.set("280uS")  # Default value
        self.dropdown_conv_time = tk.OptionMenu(root, self.selected_conversion_time, *conversion_times.keys())
        self.dropdown_conv_time.grid(row=5, column=0, padx=(220, 5), sticky="w")

        # Create a average num drop-down list
        self.avg_num_label = tk.Label(root, text="Avg Num:")
        self.avg_num_label.grid(row=5, column=0, padx=(310, 5), sticky="w")
        self.selected_average_num = tk.StringVar()
        self.selected_average_num.set("AVG_NUM_1")  # Default value
        self.dropdown_avg_num = tk.OptionMenu(root, self.selected_average_num, *average_num.keys())
        self.dropdown_avg_num.grid(row=5, column=0, padx=(370, 5), sticky="w")

        # Create ADC range drop-down list
        self.adc_range_label = tk.Label(root, text="ADC Range:")
        self.adc_range_label.grid(row=5, column=0, padx=(495, 5), sticky="w")
        self.selected_adc_range = tk.StringVar()
        self.selected_adc_range.set("RANGE_0")  # Default value
        self.dropdown_adc_range = tk.OptionMenu(root, self.selected_adc_range, *adc_range.keys())
        self.dropdown_adc_range.grid(row=5, column=0, padx=(565, 5), sticky="w")

        # Configure adc button
        self.config_button = tk.Button(root, text="Config ADC", command=self.execute_adc_configuration)
        self.config_button.grid(row=6, column=0, padx=(155, 10), sticky="w")

        # Start measuring button
        self.start_mesuring_button = tk.Button(root, text="Start Mesuring", command=self.execute_start_measuring)
        self.start_mesuring_button.grid(row=6, column=0, padx=(240, 10), sticky="w")

        # Stop measuring button
        self.stop_mesuring_button = tk.Button(root, text="Stop Mesuring", command=self.execute_stop_measuring)
        self.stop_mesuring_button.grid(row=6, column=0, padx=(340, 10), sticky="w")

        # Clear and Quit button
        self.clear_button = tk.Button(root, text="Clear Output", command=self.clear_output)
        self.clear_button.grid(row=7, column=0, padx=(155, 10), sticky="w")

        self.quit_button = tk.Button(root, text="Quit", command=self.close)
        self.quit_button.grid(row=8, column=0, padx=(155, 10), sticky="w")

        self.output_label = tk.Label(root, text="Output (Hex):")
        self.output_label.grid(row=9, column=0, padx=10, pady=10, sticky="w")

        # Adding a scroll bar to the text output
        self.output_frame = tk.Frame(root)
        self.output_frame.grid(row=10, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")
        self.output_text = tk.Text(self.output_frame, height=10, width=50, wrap=tk.NONE)
        self.output_scrollbar_y = tk.Scrollbar(self.output_frame, orient=tk.VERTICAL, command=self.output_text.yview)
        self.output_scrollbar_x = tk.Scrollbar(self.output_frame, orient=tk.HORIZONTAL, command=self.output_text.xview)
        self.output_text.configure(yscrollcommand=self.output_scrollbar_y.set, xscrollcommand=self.output_scrollbar_x.set)
        self.output_scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
        self.output_scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.output_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Configure grid layout for resizing
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        # Matplotlib figure for plotting volatge waveform
        self.figure2 = plt.Figure(figsize=(5, 2), dpi=80)
        self.ax2 = self.figure2.add_subplot(111)
        self.ax2.set_title("Volatge Waveform (V)")
        self.ax2.set_xlabel("Sample")
        self.ax2.set_ylabel("Volatage (V)")
        self.canvas2 = FigureCanvasTkAgg(self.figure2, master=root)
        self.canvas2.get_tk_widget().grid(row=11, column=0, columnspan=2, pady=10, sticky="nsew")

        # Matplotlib figure for plotting current waveform
        self.figure1 = plt.Figure(figsize=(5, 3), dpi=80)
        self.ax1 = self.figure1.add_subplot(111)
        self.ax1.set_title("Current Waveform (mA)")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("Current (mA)")
        self.canvas1 = FigureCanvasTkAgg(self.figure1, master=root)
        self.canvas1.get_tk_widget().grid(row=12, column=0, columnspan=2, pady=10, sticky="nsew")
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

        # Initialize avg_current_entry here
        self.avg_current_label = tk.Label(root, text="Average Current (mA):")
        self.avg_current_label.grid(row=13, column=0, padx=10, pady=10, sticky="w")
        self.avg_current_entry = tk.Entry(root, state="readonly")
        self.avg_current_entry.grid(row=13, column=1, padx=10, pady=10, sticky="ew")

        # Marker 1 current value
        self.marker1_label = tk.Label(root, text="Marker 1 Value:")
        self.marker1_label.grid(row=14, column=0, padx=(155, 10), sticky="w")
        self.marker1_text = tk.Entry(root)
        self.marker1_text.grid(row=14, column=1, padx=(0, 10), sticky="w")

        # Marker 2 current value
        self.marker2_label = tk.Label(root, text="Marker 2 Value:")
        self.marker2_label.grid(row=15, column=0, padx=(155, 10), sticky="w")
        self.marker2_text = tk.Entry(root)
        self.marker2_text.grid(row=15, column=1, padx=(0, 10), sticky="w")

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
        else:
            avg_current = 0  # Or another appropriate value

        self.avg_current_entry.config(state=tk.NORMAL)
        self.avg_current_entry.delete(0, tk.END)
        self.avg_current_entry.insert(0, f"{avg_current:.2f}")
        self.avg_current_entry.config(state="readonly")

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

    def execute_stop_measuring(self):
        cmd = bytearray()

        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        try:
            # Run command start measuring
            cmd = bytearray([0x08, 0x00, 0x00, 0x00])
            self.serial_port.write(cmd)
            response = self.serial_port.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return

        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.output_text.see(tk.END)

        # Update the display to show markers even without current data
        self.is_measuring = False
        self.update_current_waveform(self.current_data)

    def execute_start_measuring(self):
        cmd = bytearray()

        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        try:
            # Run command start measuring
            cmd = bytearray([0x07, 0x00, 0x00, 0x00])
            self.serial_port.write(cmd)
            response = self.serial_port.read(16)
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
        selected_conv_time = self.selected_conversion_time.get()
        hex_value = conversion_times[selected_conv_time]
        cmd.extend([0x02, 0x00, 0x00, 0x00]) # Command write config
        cmd.append(hex_value) # added cnv_time
        self.output_text.insert(tk.END, f"Selected Conversion Time: {selected_conv_time} (0x{hex_value:X})\n")

        selected_avg_num = self.selected_average_num.get()
        hex_value = average_num[selected_avg_num]
        cmd.append(hex_value) # added avg_num
        self.output_text.insert(tk.END, f"Selected Average Num: {selected_avg_num} (0x{hex_value:X})\n")

        selected_adc_range = self.selected_adc_range.get()
        hex_value = adc_range[selected_adc_range]
        cmd.append(hex_value) # added adc_range
        cmd.append(0x01)      # added avg_alert
        self.output_text.insert(tk.END, f"Selected Adc Range: {selected_adc_range} (0x{hex_value:X})\n")
        self.output_text.insert(tk.END, f"Command: {cmd}\n")

        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        try:
            # Write adc config param command
            self.serial_port.write(cmd)
            response = self.serial_port.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return

            # Run command configure INA229
            cmd = bytearray([0x04, 0x00, 0x00, 0x00])
            self.serial_port.write(cmd)
            response = self.serial_port.read(16)
            self.output_text.insert(tk.END, f"Response: {response}\n")
            if response[0] != cmd[0] or response[1] != 0x01:
                messagebox.showerror("Error", "Device respone error")
                return

        except Exception as e:
            messagebox.showerror("Error", str(e))

        self.output_text.see(tk.END)

    def connect(self):
        port = self.port_entry.get()
        baudrate = self.baudrate_entry.get()
        try:
            self.serial_port = serial.Serial(port, baudrate=int(baudrate), timeout=1)
            # messagebox.showinfo("Connection", f"Connected to {port} at {baudrate} baud")
            self.is_receiving = True
            self.receive_thread = threading.Thread(target=self.receive_data)
            self.receive_thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def disconnect(self):
        self.is_receiving = False
        if self.receive_thread:
            self.receive_thread.join()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            messagebox.showinfo("Disconnection", "Disconnected from the UART port")
        else:
            messagebox.showerror("Error", "No UART port is currently connected")

    def send_data(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Please connect to a UART port first.")
            return

        hex_input = self.input_entry.get().strip()

        try:
            # Convert space-separated hex input to bytes
            data = binascii.unhexlify(hex_input.replace(" ", ""))
            self.serial_port.write(data)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def receive_data(self):
        while self.is_receiving:
            try:
                if self.serial_port.in_waiting >= (4 + 4 + 4 * DATA_RPT_SAMPLE_SIZE * 2):
                    data = self.serial_port.read(4 + 4 + 4 * DATA_RPT_SAMPLE_SIZE * 2)

                    # Unpack the received data
                    sign, package_id = struct.unpack('<II', data[:8])

                    # Check the signature
                    if sign == SIGNATURE:
                        voltage_data = struct.unpack('<' + 'i' * DATA_RPT_SAMPLE_SIZE, data[8:8 + 4 * DATA_RPT_SAMPLE_SIZE])
                        current_data = struct.unpack('<' + 'i' * DATA_RPT_SAMPLE_SIZE, data[8 + 4 * DATA_RPT_SAMPLE_SIZE:])

                        # Append to existing data for a smooth waveform
                        self.current_data = np.append(self.current_data, current_data)
                        if len(self.current_data) > MAX_DATA_SIZE:  # Limit the size to MAX_DATA_SIZE samples for display
                            self.current_data = self.current_data[-MAX_DATA_SIZE:]

                        # Append to existing data for a smooth waveform
                        self.voltage_data = np.append(self.voltage_data, voltage_data)
                        if len(self.voltage_data) > MAX_DATA_SIZE:  # Limit the size to MAX_DATA_SIZE samples for display
                            self.voltage_data = self.voltage_data[-MAX_DATA_SIZE:]

                        # Update current waveform
                        self.update_current_waveform(self.current_data)

                        # Update volatge waveform
                        self.update_volatge_waveform(self.voltage_data)

                        #self.output_text.insert(tk.END, f"Package ID: {package_id}\n")
                        #self.output_text.insert(tk.END, f"Current (mA): {current_data[:5]}...\n")  # Display first 5 values as a preview
                        #self.output_text.see(tk.END)
            except Exception as e:
                self.is_receiving = False
                messagebox.showerror("Error", str(e))
                break

    def update_current_waveform(self, current_data):
        # Clear the axis without removing the markers
        self.ax1.clear()

        # Plot only the last MAX_DATA_SIZE samples if the data size exceeds it
        if len(current_data) > MAX_DATA_SIZE:
            self.ax1.plot(current_data[-MAX_DATA_SIZE:])
            self.original_xlim = [0, MAX_DATA_SIZE]
        else:
            self.ax1.plot(current_data)
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

    def update_volatge_waveform(self, voltage_data):
        self.ax2.clear()

        # Plot only the last MAX_DATA_SIZE samples if the data size exceeds it
        if len(voltage_data) > MAX_DATA_SIZE:
            self.ax2.plot(voltage_data[-MAX_DATA_SIZE:])
        else:
            self.ax2.plot(voltage_data)

        self.ax1.set_title("Volatge Waveform (V)")
        self.ax1.set_xlabel("Sample")
        self.ax1.set_ylabel("Voltage (mA)")
        self.canvas2.draw()

    def clear_output(self):
        self.output_text.delete('1.0', tk.END)

    def close(self):
        self.is_receiving = False
        if self.receive_thread:
            self.receive_thread.join()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = UARTApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
