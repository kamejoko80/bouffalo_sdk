#
# Reference: https://stackoverflow.com/questions/70625801/threading-reading-a-serial-port-in-python-with-a-gui
#
import tkinter as tk
from tkinter import ttk
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from serial import Serial
from serial.threaded import ReaderThread, Protocol, LineReader

DATA_RPT_SAMPLE_SIZE = 63
SIGNATURE = 0x87654321

class SerialReaderProtocolRaw(Protocol):
    tk_listener = None

    def connection_made(self, transport):
        """Called when reader thread is started"""
        if self.tk_listener is None:
            raise Exception("tk_listener must be set before connecting to the socket!")
        print("Connected, ready to receive data...")

    def data_received(self, data):
        """Called with snippets received from the serial port"""
        #print(f"Data received: {data.hex()}")  # Debug output
        self.tk_listener.root.after(0, self.tk_listener.on_data, data)

class SerialReaderProtocolLine(LineReader):
    tk_listener = None
    TERMINATOR = b'\n\r'

    def connection_made(self, transport):
        """Called when reader thread is started"""
        if self.tk_listener is None:
            raise Exception("tk_listener must be set before connecting to the socket!")
        super().connection_made(transport)
        print("Connected, ready to receive data...")

    def handle_line(self, line):
        """New line waiting to be processed"""
        # Execute our callback in tk
        self.tk_listener.root.after(0, self.tk_listener.on_data, line)

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("ADC Data Stream")

        # Initialize the buffer
        self.data_buffer = b''
        self.reader_thread = None
        self.serial_port = None

        self.fig, self.ax = plt.subplots(2, 1, figsize=(8, 6))

        self.voltage_line, = self.ax[0].plot([], [], 'r-', label="Voltage (V)")
        self.current_line, = self.ax[1].plot([], [], 'b-', label="Current (mA)")

        self.ax[0].set_title("Voltage")
        self.ax[1].set_title("Current")

        self.ax[0].set_xlim(0, DATA_RPT_SAMPLE_SIZE)
        self.ax[1].set_xlim(0, DATA_RPT_SAMPLE_SIZE)

        self.ax[0].set_ylim(-10, 10)
        self.ax[1].set_ylim(-200, 200)

        for a in self.ax:
            a.grid(True)
            a.legend()

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # COM Port Entry
        self.com_port_label = ttk.Label(root, text="COM Port:")
        self.com_port_label.pack(side=tk.LEFT, padx=(10, 0))
        self.com_port_entry = ttk.Entry(root, width=10)
        self.com_port_entry.pack(side=tk.LEFT)
        self.com_port_entry.insert(0, 'COM10')  # Default value

        self.connect_button = ttk.Button(root, text="Connect", command=self.start_serial)
        self.connect_button.pack(side=tk.LEFT, padx=(10, 0))

        self.disconnect_button = ttk.Button(root, text="Disconnect", command=self.stop_serial)
        self.disconnect_button.pack(side=tk.RIGHT, padx=(0, 10))

        # Handle closing event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def start_serial(self):
        print("Start serial")

    def stop_serial(self):
        print("Stop serial")

    def on_data(self, data):
        # Append new data to the buffer
        self.data_buffer += data

        # Check if we have enough data to process a packet
        if len(self.data_buffer) >= 512:
            # Extract the first 512 bytes
            packet = self.data_buffer[:512]
            #print(f"Data received: {packet.hex()}")  # Debug output

            # Process the packet
            self.process_packet(packet)

            # Remove the processed packet from the buffer
            self.data_buffer = self.data_buffer[512:]

            # If there's more data left in the buffer after processing one packet,
            # call on_data again to handle the next chunk.
            if len(self.data_buffer) >= 512:
                self.on_data(b'')  # Trigger another processing cycle with no new data added

    def process_packet(self, packet):
        signature, package_id = struct.unpack('<II', packet[:8])
        #print(f"Signature: {signature:#010x}, Package ID: {package_id}")  # Debug output

        if signature == SIGNATURE:
            voltage = struct.unpack('<' + 'i' * DATA_RPT_SAMPLE_SIZE, packet[8:8 + 4 * DATA_RPT_SAMPLE_SIZE])
            current = struct.unpack('<' + 'i' * DATA_RPT_SAMPLE_SIZE, packet[8 + 4 * DATA_RPT_SAMPLE_SIZE:])
            #print(f"Voltage: {voltage}\nCurrent: {current}")  # Debug output
            self.update_graph(voltage, current, package_id)
        else:
            print("Invalid signature detected.")

    def update_graph(self, voltage, current, package_id):
        x = np.arange(DATA_RPT_SAMPLE_SIZE)
        self.voltage_line.set_data(x, np.array(voltage))  # Displaying voltage directly in volts
        self.current_line.set_data(x, np.array(current))  # Displaying current directly in mA

        self.ax[0].relim()
        self.ax[0].autoscale_view()
        self.ax[1].relim()
        self.ax[1].autoscale_view()

        self.canvas.draw()

    def on_close(self):
        self.root.destroy()  # Destroy the Tkinter root window

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    # Set listener to our app instance
    SerialReaderProtocolRaw.tk_listener = app
    # Initiate serial port
    serial_port = Serial("COM10")
    # Initiate ReaderThread
    reader = ReaderThread(serial_port, SerialReaderProtocolRaw)
    # Start reader
    reader.start()
    # Run app
    app.run()
