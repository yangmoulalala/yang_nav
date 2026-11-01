#!/usr/bin/env bash
# 全模块仿真回归：单/双雷达 ×（建图+回环、先验定位+绑架、原地旋转）+ 双雷达标定。
#   tools/sim_tests/run_all.sh [OUT_ROOT]      # 默认 /tmp/zt_all
# 各场景串行运行（同一时刻只有一个 Gazebo），结束后打印汇总。
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=${1:-/tmp/zt_all}
rm -rf "$ROOT"; mkdir -p "$ROOT"

clean() {  # 清掉上一场可能残留的仿真/定位进程，避免 /clock 冲突
  for p in $(pgrep -f 'gz sim -r|parameter_bridge|zlio_node|backend_node|merge_node|lidar_calibration/calibrate'); do
    kill "$p" 2>/dev/null
  done
  sleep 3
}

run() {  # run NAME LIDAR MODE MAP LAPS [driver_mode] [CALIB]
  local name=$1 lidar=$2 mode=$3 map=$4 laps=$5 dmode=${6:-} calib=${7:-0}
  clean
  echo "=== $name (lidar=$lidar mode=$mode)"
  LIDAR=$lidar CALIB=$calib "$HERE/zt_run.sh" "$mode" "$ROOT/$name" "$map" "$laps" $dmode
}

for L in single dual; do
  run "${L}_mapping" $L mapping "$ROOT/map_$L" 2
  run "${L}_localization" $L localization "$ROOT/map_$L" 1
  run "${L}_spin" $L mapping "$ROOT/spinmap_$L" 0 spin
done
run dual_calibration dual mapping "$ROOT/calibmap" 0 "" 1
clean

echo
echo '################ SUMMARY ################'
for d in "$ROOT"/*/; do
  n=$(basename "$d"); [ -f "$d/bringup.log" ] || continue
  echo "--- $n"
  [ -f "$d/result.json" ] && python3 "$HERE/zt_eval.py" "$d/result.json" 2>&1 | tail -12
  echo "loops accepted: $(grep -c 'loop scan-to-map accepted' "$d/bringup.log")  " \
       "IMU interrupts: $(grep -c 'IMU input interrupted' "$d/bringup.log")  " \
       "errors: $(grep -c -E '\[ERROR\]|\[FATAL\]|process has died' "$d/bringup.log")"
  grep -E '\[ERROR\]|\[FATAL\]|process has died' "$d/bringup.log" | head -3
  grep -h 'SERVICE' "$d/driver.log" 2>/dev/null
  if [ -f "$d/calib.log" ]; then
    echo "calib accepted: $(grep -c 'accepted' "$d/calib.log")  rejected: $(grep -c 'rejected' "$d/calib.log")"
    grep -E 'calibrated|failed' "$d/calib.log" | tail -2
    [ -f "$d/extrinsic.yaml" ] && cat "$d/extrinsic.yaml"
  fi
done
