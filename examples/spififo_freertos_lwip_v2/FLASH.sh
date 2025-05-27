#!/bin/bash

ENV_FILE=".bl616_ports.conf"

# Load saved COM port values if the file exists
if [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
else
    echo "Please enter COM port for:"
    read -p "  MCU MODULE A (e.g., COM5): " COM_A
    read -p "  MCU MODULE B (e.g., COM6): " COM_B

    # Save to env file
    echo "COM_A=$COM_A" > "$ENV_FILE"
    echo "COM_B=$COM_B" >> "$ENV_FILE"
fi

echo "Flash options:

    1) MCU MODULE A
    2) MCU MODULE B

Enter your choice:"

read -p "Choice: " choice

case "$choice" in
    1)
        echo "Flashing MCU MODULE A on $COM_A..."
        make flash CHIP=bl616 COMX=$COM_A
        ;;
    2)
        echo "Flashing MCU MODULE B on $COM_B..."
        make flash CHIP=bl616 COMX=$COM_B
        ;;
    *)
        echo "Invalid choice. Please select 1 or 2."
        exit 1
        ;;
esac