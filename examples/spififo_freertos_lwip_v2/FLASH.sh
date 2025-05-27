#!/bin/bash

echo "Flash options:

    1) MCU MODULE A
    2) MCU MODULE B

Enter your choice:"

read -p "Choice: " choice

case "$choice" in
    1)
        echo "Flashing MCU MODULE A..."
        make flash CHIP=bl616 COMX=COM5
        ;;
    2)
        echo "Flashing MCU MODULE B..."
        make flash CHIP=bl616 COMX=COM5
        ;;
    *)
        echo "Invalid choice. Please select 1 or 2."
        exit 1
        ;;
esac
