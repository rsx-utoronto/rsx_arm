#! /bin/bash

# Copied from the old rsx-rover repo; used to setup the arm's CAN network

#echo "Adding CAN network"
#sudo ip link add dev can0 type can
sudo ip link set down can0
echo "Setting CAN network bitrate and queue length"
sudo ip link set can0 type can bitrate 1000000
sudo ifconfig can0 txqueuelen 1000
echo "Enabling the CAN network"
sudo ip link set down can0
sudo ip link set up can0