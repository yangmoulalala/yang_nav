// Direction-constrained zones from terrain_export.region_zones, shared by the path shaper and tracker.
// The semantic type selects default tuning; one_way_dir / allow_flip / recentre define traversal policy.
//
// `warning: direction varies inside the region` (a multi-direction patch such as a fortress face) marks the
// zone axis unreliable: the shaper leaves those to the planner instead of locking a heading the terrain
// does not have. min_cells drops narrow-passage fragments; slope_min_cells applies to open directional zones.
#pragma once

#include <string>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"

namespace nav_behaviors
{

struct ZoneLoad
{
  std::vector<Zone> zones;
  std::vector<std::string> dropped;   // "slope_9 (12 cells)"
  int tunnels{0}, directional{0}, ambiguous{0};
};

// `min_cells` applies to tunnels, `slope_min_cells` to other directional terrain types.
ZoneLoad loadZones(
  const std::string & file, int min_cells, int slope_min_cells,
  const std::vector<std::string> & types = {
    "tunnel", "slope", "step_l1", "step_l2", "step_high", "fly_slope", "undulation"});

// Narrow tunnels only, for callers that use the original tunnel-only config.
std::vector<Zone> loadTunnels(const std::string & file, int min_cells, std::vector<std::string> * dropped = nullptr);

}  // namespace nav_behaviors
