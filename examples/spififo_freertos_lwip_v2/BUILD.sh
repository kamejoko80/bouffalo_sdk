#!/bin/bash

echo "Build options:

    1) MCU MODULE A
    2) MCU MODULE B

Enter your choice:"

read -p "Choice: " choice

case "$choice" in
    1)
        echo "Building with MCU MODULE A..."
        make ninja CHIP=bl616 BOARD=bl616dk FPGA_MODULE=GW1N_LV1 MCU_MODULE=A
        ;;
    2)
        echo "Building with MCU MODULE B..."
        make ninja CHIP=bl616 BOARD=bl616dk FPGA_MODULE=GW1N_LV1 MCU_MODULE=B
        ;;
    *)
        echo "Invalid choice. Please select 1 or 2."
        exit 1
        ;;
esac

