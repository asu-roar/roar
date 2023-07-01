#!/bin/bash

export SUDO_ASKPASS=/home/nvidia/roar_ws/src/roscan/roscan/config/helper.sh

# Open folder in terminal and run using ./vcan.sh

# Make sure the script runs with super user privileges.
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 type can bitrate 125000 restart-ms 200

ip link set up can0

ip -details -statistics link show can0
