SDK_DEMO_PATH ?= .
BL_SDK_BASE ?= $(SDK_DEMO_PATH)/../..

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-
MCU_MODULE ?= A
FPGA_MODULE ?= GW1N_LV1

# add custom cmake definition
cmake_definition+= -DMCU_MODULE=$(MCU_MODULE)
cmake_definition+= -DFPGA_MODULE=$(FPGA_MODULE)

include $(BL_SDK_BASE)/project.build
