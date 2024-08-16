#
# Reference: https://stackoverflow.com/questions/70625801/threading-reading-a-serial-port-in-python-with-a-gui
#
import struct
import tkinter as tk

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
        self.tk_listener.after(0, self.tk_listener.on_data, data)

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
        self.tk_listener.after(0, self.tk_listener.on_data, line)

class MainFrame(tk.Frame):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.listbox = tk.Listbox(self)
        self.listbox.pack()
        self.pack()
        self.data_buffer = b''

    def on_data(self, data):
        #print("Called from tk Thread:", data)
        #self.listbox.insert(tk.END, data)

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
            self.listbox.insert(tk.END, f"Package ID: {package_id}\n")
            self.listbox.insert(tk.END, f"Current (mA): {current}\n")
            self.listbox.insert(tk.END)
        else:
            print("Invalid signature detected.")

if __name__ == '__main__':
    app = tk.Tk()

    main_frame = MainFrame()
    # Set listener to our reader
    SerialReaderProtocolRaw.tk_listener = main_frame
    # Initiate serial port
    serial_port = Serial("COM14")
    # Initiate ReaderThread
    reader = ReaderThread(serial_port, SerialReaderProtocolRaw)
    # Start reader
    reader.start()

    app.mainloop()

