#!/usr/bin/env bash
# ZLIO 后端仿真回归测试：建图 + 回环 / 先验定位 + 绑架重定位。
#   tools/sim_tests/zt_run.sh mapping      OUT_DIR MAP_DIR [laps]
#   tools/sim_tests/zt_run.sh localization OUT_DIR MAP_DIR [laps] [driver_mode] [extra]
#     driver_mode overrides mapping/localize (e.g. spin); extra=spin turns at every waypoint
# 仿真与 bringup 在同一进程组中启动，驱动脚本结束后只停止该进程组。
HERE=$(cd "$(dirname "$0")" && pwd)
MODE=$1; OUT=$2; MAP=$3; LAPS=${4:-2}
mkdir -p "$OUT"
cd ~/yang_nav
# 非交互 shell 不读 .bashrc：显式指定独显，否则 gpu_lidar 回退软件渲染
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# LIDAR=single|dual（默认 dual）；CALIB=1 时同时启动双雷达标定，结果写 $OUT/extrinsic.yaml
LIDAR=${LIDAR:-dual}
CALIB_CMD=""
if [ "${CALIB:-0}" = 1 ]; then
  CALIB_CMD="(sleep 15; ros2 launch bringup calibration.launch.py platform:=sim output_file:=$OUT/extrinsic.yaml > $OUT/calib.log 2>&1) &"
fi
setsid bash -c "ros2 launch sentry_gazebo sim.launch.py > $OUT/sim.log 2>&1 & sleep 12; $CALIB_CMD ros2 launch bringup sim.launch.py lidar:=$LIDAR mode:=$MODE map_dir:=$MAP rviz:=false > $OUT/bringup.log 2>&1; wait" &
PGID=$!
trap 'kill -INT -- -$PGID 2>/dev/null; sleep 4; kill -KILL -- -$PGID 2>/dev/null' EXIT
sleep 20
if [ "$MODE" = mapping ]; then DMODE=mapping; else DMODE=localize; fi
[ "${CALIB:-0}" = 1 ] && DMODE=calib
DMODE=${5:-$DMODE}
timeout 600 python3 "$HERE/zt_driver.py" $DMODE "$OUT/result.json" "$LAPS" ${6:-} > "$OUT/driver.log" 2>&1
echo "driver exit=$?"
