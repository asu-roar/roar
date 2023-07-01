#!/bin/bash

[ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"


ssh nvidia@192.168.0.254
