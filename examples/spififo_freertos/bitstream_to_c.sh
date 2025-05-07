#!/bin/bash

# Convert GW1N bitstream to C header file
# Fof the project.bin, refer to the repo litex-soc-builder and the design file in:
# litex-soc-builder/custom_projects/test_spi_fifo_gw1n_fpga_evb.py => For static clock frequency implementation
# litex-soc-builder/custom_projects/test_spi_fifo_gw1n_dynamic_clk_fpga_evb.py => For dynamic clock frequency implementation

#./tools/bin2uint8_t.exe project.bin gw1n_image.h
./tools/bin2uint8_t.exe //wsl.localhost/Ubuntu/home/henry/Workspace/Litex/litex-soc-builder/custom_projects/build/impl/pnr/project.bin gw1n_image.h


