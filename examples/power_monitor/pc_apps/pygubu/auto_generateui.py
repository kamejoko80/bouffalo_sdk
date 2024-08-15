#!/usr/bin/python3
import pathlib
import tkinter as tk
import pygubu

PROJECT_PATH = pathlib.Path(__file__).parent
PROJECT_UI = PROJECT_PATH / "power_monitor.ui"
RESOURCE_PATHS = [PROJECT_PATH]


class auto_generateUI:
    def __init__(self, master=None, on_first_object_cb=None):
        self.builder = pygubu.Builder(
            on_first_object=on_first_object_cb)
        self.builder.add_resource_paths(RESOURCE_PATHS)
        self.builder.add_from_file(PROJECT_UI)
        # Main widget
        self.mainwindow: tk.Toplevel = self.builder.get_object(
            "toplevel", master)
        self.builder.connect_callbacks(self)

    def run(self):
        self.mainwindow.mainloop()

    def connect(self):
        pass

    def disconnect(self):
        pass

    def send_data(self):
        pass

    def execute_adc_configuration(self):
        pass

    def on_scale_change(self, scale_value):
        pass

    def on_set_vbat_value(self):
        pass

    def execute_start_measuring(self):
        pass

    def execute_stop_measuring(self):
        pass

    def clear_output(self):
        pass

    def close(self):
        pass


if __name__ == "__main__":
    app = auto_generateUI()
    app.run()
