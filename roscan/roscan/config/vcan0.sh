#!/bin/bash

export SUDO_ASKPASS=/home/belal/roar_ws/src/roscan/roscan/config/helper.sh

# Open folder in terminal and run using ./vcan.sh

# Make sure the script runs with super user privileges.
[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

# Load the kernel module.
modprobe vcan

# Create the virtual CAN interface.
ip link add dev vcan0 type vcan

# Bring the virtual CAN interface online.
ip link set up vcan0

# BShow virtual CAN interface details.
ip -details -statistics link show vcan0
