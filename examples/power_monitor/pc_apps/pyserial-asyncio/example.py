import tkinter as tk
import asyncio
import serial
import serial_asyncio

class SerialReader(asyncio.Protocol):
    def __init__(self, display_callback):
        self.display_callback = display_callback

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self, data):
        # Handle incoming data
        self.display_callback(data.decode('utf-8'))

    def connection_lost(self, exc):
        # Handle connection loss
        print('Serial connection lost')

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Async Serial Reader")

        self.text = tk.Text(root, height=20, width=50)
        self.text.pack()

        self.connect_button = tk.Button(root, text="Connect", command=self.start_serial)
        self.connect_button.pack()

        self.disconnect_button = tk.Button(root, text="Disconnect", command=self.stop_serial)
        self.disconnect_button.pack()

        self.serial_task = None

    def start_serial(self):
        port = 'COM3'  # Replace with your port
        baudrate = 9600  # Replace with your baudrate

        loop = asyncio.get_event_loop()
        coro = serial_asyncio.create_serial_connection(
            loop, lambda: SerialReader(self.display_data), port, baudrate
        )
        self.serial_task = loop.create_task(coro)

    def stop_serial(self):
        if self.serial_task:
            self.serial_task.cancel()
            self.serial_task = None

    def display_data(self, data):
        # Update the Tkinter text widget with the received data
        self.text.insert(tk.END, data)
        self.text.see(tk.END)  # Scroll to the end

    def run(self):
        # Integrate asyncio event loop with Tkinter
        self.root.after(100, self.poll_loop)
        self.root.mainloop()

    def poll_loop(self):
        loop = asyncio.get_event_loop()
        loop.call_soon(loop.stop)
        loop.run_forever()
        self.root.after(100, self.poll_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    app.run()

