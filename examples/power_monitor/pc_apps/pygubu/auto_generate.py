#!/usr/bin/python3
import pathlib
import tkinter as tk
import pygubu
from auto_generateui import auto_generateUI


class auto_generate(auto_generateUI):
    def __init__(self, master=None, on_first_object_cb=None):
        super().__init__(master, on_first_object_cb=None)


if __name__ == "__main__":
    app = auto_generate()
    app.run()
