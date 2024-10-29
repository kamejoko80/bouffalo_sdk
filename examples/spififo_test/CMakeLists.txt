cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

sdk_add_include_directories(.)

sdk_add_compile_definitions(-DMCU_MODULE_${MCU_MODULE})
sdk_add_compile_definitions(-DFPGA_MODULE_${FPGA_MODULE})

target_sources(app PRIVATE gw1n.c lmcxo2.c cdc_acm_template.c spi_fifo_test.c)
sdk_set_main_file(main.c)

project(spififo_test)
