#!/usr/bin/env bash
# ZLIO 后端仿真回归测试：建图 + 回环 / 先验定位 + 绑架重定位。
#   tools/sim_tests/zt_run.sh mapping      OUT_DIR MAP_DIR [laps]
#   tools/sim_tests/zt_run.sh localization OUT_DIR MAP_DIR [laps] [driver_mode] [flags]
#     driver_mode overrides mapping/localize (e.g. spin); flags: comma-separated
#     spin / initspin / earlyspin / spawnspin / kidnapspin (see zt_driver.py); flags require driver_mode
#   env: LIDAR=single|dual  ARENA=rmul|rmuc  TEAM=red|blue  CALIB=1
#        SIM_UWB=true|false（仿真是否启动 uwb_sim，仅 RMUC；后端 SUSPECT/LOST 恢复只用 UWB 位置先验，关掉后不会恢复）
# 仿真与 bringup 在同一进程组中启动，驱动脚本结束后只停止该进程组。
HERE=$(cd "$(dirname "$0")" && pwd)
if (( $# < 3 )); then
  echo 'Usage: zt_run.sh MODE OUT_DIR MAP_DIR [laps] [driver_mode] [flags]' >&2
  exit 2
fi
MODE=$1; OUT=$(realpath -m "$2"); MAP=$(realpath -m "$3"); LAPS=${4:-2}
mkdir -p "$OUT"
cd "$HERE/../.." || exit 2
# 非交互 shell 不读 .bashrc：显式指定独显，否则 gpu_lidar 回退软件渲染
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# LIDAR=single|dual（默认 dual）；CALIB=1 时同时启动双雷达标定，结果写 $OUT/extrinsic.yaml
# ARENA=rmul|rmuc（默认 rmul）
LIDAR=${LIDAR:-dual}
export ARENA=${ARENA:-rmul}
# TEAM=red|blue（默认空 = 场地原出生点：RMUL 红方、RMUC 蓝方）：仿真出生在该方半场，uwb_sim 发布到 /uwb_pose/team
export TEAM=${TEAM:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87} GZ_PARTITION=${GZ_PARTITION:-zt_$$}
setsid bash -c '
  out=$1
  ros2 launch sentry_gazebo sim.launch.py chassis:=false "arena:=$ARENA" ${TEAM:+"team:=$TEAM"} "uwb:=${SIM_UWB:-true}" > "$out/sim.log" 2>&1 &
  sleep 12
  if [[ ${CALIB:-0} == 1 ]]; then
    (sleep 15; ros2 launch bringup calibration.launch.py platform:=sim "output_file:=$out/extrinsic.yaml" > "$out/calib.log" 2>&1) &
  fi
  ros2 launch bringup sim.launch.py "lidar:=$2" "mode:=$3" "map_dir:=$4" rviz:=false > "$out/bringup.log" 2>&1
  wait
' bash "$OUT" "$LIDAR" "$MODE" "$MAP" &
PGID=$!
trap 'kill -INT -- -$PGID 2>/dev/null; sleep 4; kill -KILL -- -$PGID 2>/dev/null' EXIT
sleep 20
if [ "$MODE" = mapping ]; then DMODE=mapping; else DMODE=localize; fi
[ "${CALIB:-0}" = 1 ] && DMODE=calib
DMODE=${5:-$DMODE}
timeout 600 python3 "$HERE/zt_driver.py" "$DMODE" "$OUT/result.json" "$LAPS" ${6:+"$6"} > "$OUT/driver.log" 2>&1
rc=$?
echo "driver exit=$rc"
exit "$rc"
