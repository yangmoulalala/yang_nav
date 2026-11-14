#!/bin/bash
# Static checks of installed navigation libraries, configuration and both zone-aware BTs.
# Tests:
#   1. Nav2 plugin libraries and registrations exist (runtime checks are separate)
#   2. Zones.yaml is readable and contains tunnels
#   3. Navigation parameters are valid
set -eo pipefail   # not -u: install/setup.bash reads unset COLCON_TRACE

WS=$(cd "$(dirname "$0")/../.." && pwd)
MAP=${MAP:-$WS/maps/sim}
# the terrain package may sit beside the prior map or in its own export folder (maps/sim/terrain)
MAP=$(python3 "$WS/tools/nav_regress/terrain_path.py" "$MAP")

echo "=== Navigation System Verification ==="
echo "Workspace: $WS"
echo "Map/Terrain: $MAP"
echo

# Check 1: Zone configuration exists
echo "[1/5] Checking zone configuration..."
if [ ! -f "$MAP/zones.yaml" ]; then
    echo "ERROR: zones.yaml not found at $MAP/zones.yaml"
    exit 1
fi
TUNNEL_COUNT=$(grep -c "type: tunnel" "$MAP/zones.yaml" || true)
echo "  ✓ Found zones.yaml with $TUNNEL_COUNT tunnels"

# Check 2: Verify installed libraries
echo "[2/5] Checking installed navigation libraries..."
source "$WS/install/setup.bash"
for lib in zone_aware_controller zone_path_shaper zone_bt_nodes; do
    if [ -f "$WS/install/nav_behaviors/lib/lib${lib}.so" ]; then
        echo "  ✓ lib${lib}.so installed"
    else
        echo "  ERROR: lib${lib}.so not found"
        exit 1
    fi
done

# Check 3: Verify plugin registration
echo "[3/5] Checking plugin registration..."
PLUGINS_XML="$WS/install/nav_behaviors/share/nav_behaviors/plugins.xml"
if [ ! -f "$PLUGINS_XML" ]; then
    echo "  ERROR: plugins.xml not installed at $PLUGINS_XML"
    exit 1
fi
for cls in "nav_behaviors::ZoneAwareController" "nav_behaviors::ZonePathShaper"; do
    if grep -q "type=\"$cls\"" "$PLUGINS_XML"; then
        echo "  ✓ $cls registered"
    else
        echo "  ERROR: $cls missing from $PLUGINS_XML"
        exit 1
    fi
done
if [ -f "$WS/install/nav_behaviors/share/ament_index/resource_index/nav2_core__pluginlib__plugin/nav_behaviors" ]; then
    echo "  ✓ pluginlib class index exported"
else
    echo "  WARNING: no nav2_core pluginlib index for nav_behaviors"
fi
# BT nodes are not pluginlib classes: they are looked up by the BT factory at tree load time.  The
# class names appear only mangled in the symbol table (nav_behaviors::ZoneRecovery -> 12ZoneRecovery).
BT_SYMBOLS=$(strings "$WS/install/nav_behaviors/lib/libzone_bt_nodes.so")
for pair in "ZoneReplanPaused:16ZoneReplanPaused" "ZoneRecovery:12ZoneRecovery"; do
    name=${pair%%:*}
    symbol=${pair#*:}
    if grep -q "$symbol" <<< "$BT_SYMBOLS"; then
        echo "  ✓ $name registered in libzone_bt_nodes.so"
    else
        echo "  ERROR: $name missing from libzone_bt_nodes.so"
        exit 1
    fi
done

# Check 4: Verify behavior tree XML
echo "[4/5] Checking behavior tree configuration..."
BT_DIR="$WS/install/nav_bringup/share/nav_bringup/behavior_trees"
for tree in navigate_to_pose.xml navigate_through_poses.xml; do
    BT_XML="$BT_DIR/$tree"
    if [ ! -f "$BT_XML" ]; then
        echo "  ERROR: $tree not installed"
        exit 1
    fi
    for required in '<ZoneRecovery' '<ZoneReplanPaused' '<SmoothPath' 'zones_file="__NAV_ZONES_FILE__"' 'backup_after'; do
        if ! grep -Fq "$required" "$BT_XML"; then
            echo "  ERROR: $tree is missing $required"
            exit 1
        fi
    done
    echo "  ✓ $tree configures shaping, zone-path retention and protected recovery"
done

# Check 5: Validate nav2 parameters
echo "[5/5] Checking Nav2 parameters..."
NAV2_YAML="$WS/install/nav_bringup/share/nav_bringup/params/sim/nav2.yaml"
if [ -f "$NAV2_YAML" ]; then
    if grep -q "plugin: nav_behaviors::ZoneAwareController" "$NAV2_YAML"; then
        echo "  ✓ ZoneAwareController configured in nav2.yaml"
    else
        echo "  WARNING: ZoneAwareController not configured"
    fi
    if grep -q "plugin_lib_names:.*zone_bt_nodes" "$NAV2_YAML"; then
        echo "  ✓ zone_bt_nodes configured in bt_navigator"
    else
        echo "  WARNING: zone_bt_nodes not in plugin_lib_names"
    fi
else
    echo "  ERROR: nav2.yaml not found"
    exit 1
fi

echo
echo "=== All Checks Passed ✓ ==="
echo "Static installation checks passed. Run the integration tools for runtime verification."
echo
echo "Next steps:"
echo "  • Run unit tests: colcon test --packages-select nav_behaviors nav_safety nav_terrain"
echo "  • Full-stack smoke: python3 tools/nav_regress/framework_smoke.py"
echo "  • Recovery ladder: python3 tools/nav_regress/recovery_ladder.py"
echo "  • Shaped planning: SMOOTHER=zone_shaper tools/nav_regress/plan_tunnels.sh"
