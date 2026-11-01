#!/bin/bash
set -e
cd ~/yang_nav
rm -f tools/.tmp_check.sh tools/.tmp_commit_msg
find src/z-localization tools -name __pycache__ -not -path '*/reference/*' -exec rm -rf {} + 2>/dev/null || true
echo '=== branch'; git branch --show-current
echo '=== status'; git status --short --untracked-files=all
TRAILER='Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>'
commit() { # commit MSG PATHS...
  local msg=$1; shift
  git add -A -- "$@"
  if git diff --cached --quiet; then echo "skip: $msg"; return; fi
  git commit -q -F - <<EOF
$msg

$TRAILER
EOF
  git log --oneline -1
}
commit 'frontend: read standard ROS params, LiDAR extrinsic from TF, publish world->base' src/z-localization/frontend
commit 'backend: strict ROS params, remapped topics, per-stamp body extrinsic from TF' src/z-localization/backend
commit 'lidar_fusion, lidar_calibration: strict params, remapped topics, example configs and launches' src/z-localization/lidar_fusion src/z-localization/lidar_calibration
commit 'bringup: shared localization pipeline, single-source sim/real configs, driver and joint-state configs' src/z-localization/bringup src/sentry_description src/sentry_gazebo
commit 'docs, tools: update READMEs and guides, param key check, sim regression for single/dual and calibration' src/z-localization/README.md docs tools
echo '=== remaining'; git status --short --untracked-files=all
echo '=== merge into main'
git checkout -q main
git merge --ff-only test/zlio-backend || git merge --no-edit test/zlio-backend
git log --oneline -8
git branch --show-current
rm -f tools/.tmp_commit.sh
