#!/usr/bin/env bash
# Local-only, pinned gtsam_points build using the same GTSAM ABI as backend.
set -e -o pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
root="$ws/.deps/offline_optimizer"
revision=00f9fca52b8a24abce3d7ce04f290f135a7615e6
mkdir -p "$root"
if [[ ! -d "$root/gtsam_points/.git" ]]; then
  git clone --depth 1 --branch v1.2.0 https://github.com/koide3/gtsam_points.git "$root/gtsam_points"
fi
if [[ "$(git -C "$root/gtsam_points" rev-parse HEAD)" != "$revision" ]]; then
  echo "Unexpected gtsam_points revision in $root/gtsam_points; expected $revision" >&2
  exit 1
fi
if [[ -n "$(git -C "$root/gtsam_points" status --porcelain)" ]]; then
  echo "gtsam_points source has local changes; refusing to build an unrecorded version" >&2
  exit 1
fi
cmake -S "$root/gtsam_points" -B "$root/build-gtsam-points" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/install" \
  -DGTSAM_DIR=/opt/ros/jazzy/lib/cmake/GTSAM \
  -DGTSAM_UNSTABLE_DIR=/opt/ros/jazzy/lib/cmake/GTSAM_UNSTABLE \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_OPENMP=ON -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DBUILD_TESTS=OFF -DBUILD_DEMO=OFF -DBUILD_EXAMPLE=OFF -DBUILD_TOOLS=OFF
cmake --build "$root/build-gtsam-points" --parallel "${JOBS:-2}"
cmake --install "$root/build-gtsam-points"
mkdir -p "$root/install/share/gtsam_points"
install -m 644 "$root/gtsam_points/LICENSE" "$root/install/share/gtsam_points/LICENSE"
printf '%s\n' "$revision" > "$root/install/share/gtsam_points/SOURCE_REVISION"
printf 'Installed gtsam_points 1.2.0 (%s) in %s\n' "$revision" "$root/install"
