#!/bin/bash
cd /home/yang/yang_nav
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92 GZ_PARTITION=yaw_nav_repro_20261113 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export OUT=/home/yang/yang_nav/reports/navigation/2026-11-13/yaw_diagnosis/repro
mkdir -p "$OUT"
python3 -u /tmp/navdev/record_yaw_chain.py "$OUT" > "$OUT/recorder.log" 2>&1 &
REC=$!
trap 'kill -INT "$REC" 2>/dev/null || true' EXIT
GUI=false RVIZ=false GOAL_FRAME=map GOAL_TIMEOUT=100 HOLD=3 bash /tmp/navdev/run_nav_trace.sh '1.87 -1.70 0' '7.69 -3.24 0' > "$OUT/runner.log" 2>&1