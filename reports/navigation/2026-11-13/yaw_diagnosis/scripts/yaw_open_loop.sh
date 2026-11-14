#!/bin/bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=yaw_diagnosis_20261113 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
python3 -u /tmp/navdev/yaw_open_loop.py
