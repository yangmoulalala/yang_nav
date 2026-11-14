#!/bin/bash
# Bring up the full sim stack (gz + localization + navigation) with the navigation RViz and keep it
# running for manual testing: set goals with "2D Goal Pose" / "Nav2 Goal" in RViz.
#
#   tools/nav_regress/sim_rviz.sh            # runs until Ctrl-C (everything is stopped on exit)
#   GUI=true tools/nav_regress/sim_rviz.sh   # also open the Gazebo window
#
# Logs: $OUT (default /tmp/nav_manual): sim.log, loc.log, nav.log, truth.log (gz ground truth).
# Localization diagnostics default on; LOC_REC=false disables the independent trace.
# Note: run_nav.sh refuses to start over a detected navigation/simulation stack.
# On exit it stops only the process groups started by this run.
HERE=$(cd "$(dirname "$0")" && pwd)
export OUT=${OUT:-/tmp/nav_manual} RVIZ=true HOLD=${HOLD:-86400} LOC_REC=${LOC_REC:-true}
exec bash "$HERE/run_nav.sh"
