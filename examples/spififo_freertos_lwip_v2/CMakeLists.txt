cmake_minimum_required(VERSION 3.15)

include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

sdk_add_compile_definitions(-DMCU_MODULE_${MCU_MODULE})
sdk_add_compile_definitions(-DFPGA_MODULE_${FPGA_MODULE})

sdk_add_include_directories(.)

target_sources(app PRIVATE
               gw1n.c
               spi_fifo.c
               ethernetif.c
               ping.c
               )

sdk_set_main_file(main.c)

project(spififo_freertos_lwip_v2)
