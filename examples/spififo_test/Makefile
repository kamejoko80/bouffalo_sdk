SDK_DEMO_PATH ?= .
BL_SDK_BASE ?= $(SDK_DEMO_PATH)/../..

export BL_SDK_BASE

CHIP ?= bl616
BOARD ?= bl616dk
CROSS_COMPILE ?= riscv64-unknown-elf-
MCU_MODULE ?= A

# add custom cmake definition
cmake_definition+= -DMCU_MODULE=$(MCU_MODULE)

include $(BL_SDK_BASE)/project.build
