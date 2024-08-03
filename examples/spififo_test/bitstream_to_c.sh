#!/bin/bash

# Convert GW1N bitstream to C header file
# Fof the project.bin, refer to the repo litex-soc-builder and the design file in:
# litex-soc-builder/custom_projects/power_monitor_gw1n_lv1.py

./tools/bin2uint8_t.exe project.bin gw1n_image.h

