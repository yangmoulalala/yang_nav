#!/usr/bin/env bash
# 全模块仿真回归：单/双雷达 ×（建图+回环、先验定位+绑架、原地旋转）+ 双雷达标定（RMUL），
# 以及 RMUC 单/双雷达 ×（建图、先验定位+绑架）。
#   tools/sim_tests/run_all.sh [OUT_ROOT]      # 默认 /tmp/zt_all
# 各场景串行运行（同一时刻只有一个 Gazebo），结束后打印汇总。
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(realpath -m "${1:-/tmp/zt_all}")
mkdir -p "$ROOT" || exit 2
if [[ -n $(find "$ROOT" -mindepth 1 -maxdepth 1 -print -quit) ]]; then
  echo "Output directory already contains results: $ROOT. Choose an empty directory." >&2
  exit 2
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87} GZ_PARTITION=${GZ_PARTITION:-zt_all_$$}
failures=0
printf 'scenario\texit_code\n' > "$ROOT/run_status.tsv"

run() {  # run NAME LIDAR MODE MAP LAPS [driver_mode] [CALIB] [flags]
  local name=$1 lidar=$2 mode=$3 map=$4 laps=$5 dmode=${6:-} calib=${7:-0} flags=${8:-}
  # 场地由调用处的 ARENA 变量给出（默认 rmul）
  local arena=${ARENA:-rmul}
  echo "=== $name (arena=$arena lidar=$lidar mode=$mode${flags:+ flags=$flags})"
  if [ -n "$flags" ]; then  # 传 flags 时 driver_mode 必须显式给出
    [ -n "$dmode" ] || { [ "$mode" = mapping ] && dmode=mapping || dmode=localize; }
    LIDAR=$lidar CALIB=$calib ARENA=$arena "$HERE/zt_run.sh" "$mode" "$ROOT/$name" "$map" "$laps" "$dmode" "$flags"
  else
    LIDAR=$lidar CALIB=$calib ARENA=$arena "$HERE/zt_run.sh" "$mode" "$ROOT/$name" "$map" "$laps" $dmode
  fi
  local rc=$?
  printf '%s\t%s\n' "$name" "$rc" >> "$ROOT/run_status.tsv"
  if (( rc != 0 )); then failures=$((failures + 1)); fi
}

# 单雷达（侧装 MID-360）：初始化期间慢转；建图另在出生点慢转一圈（见 README“单雷达”）
for L in single dual; do
  if [ $L = single ]; then F=earlyspin; FM=earlyspin,spawnspin; else F=; FM=; fi
  run "${L}_mapping" $L mapping "$ROOT/map_$L" 2 "" 0 "$FM"
  run "${L}_localization" $L localization "$ROOT/map_$L" 1 "" 0 "$F"
  run "${L}_spin" $L mapping "$ROOT/spinmap_$L" 0 spin 0 "$F"
done
run dual_calibration dual mapping "$ROOT/calibmap" 0 "" 1

# RMUC 全场：建图；同一地图先验定位 + 绑架
ARENA=rmuc
for L in single dual; do
  if [ $L = single ]; then F=earlyspin; FM=earlyspin,spawnspin; else F=; FM=; fi
  run "rmuc_${L}_mapping" $L mapping "$ROOT/map_rmuc_$L" 2 "" 0 "$FM"
  run "rmuc_${L}_localization" $L localization "$ROOT/map_rmuc_$L" 1 "" 0 "$F"
done
ARENA=rmul

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
echo "Scenario process failures: $failures (see $ROOT/run_status.tsv; accuracy still requires evaluation)"
exit "$((failures > 0))"
