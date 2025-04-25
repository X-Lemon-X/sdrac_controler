#!/bin/bash

sudo ip link set can0 txqueuelen 1000 
sudo ip link set dev can0 up type can bitrate 1000000