// Tunnel zones from a terrain package's zones.yaml (terrain_export.region_zones), shared by the plugins.
#pragma once

#include <string>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"

namespace nav_behaviors
{

// Tunnel zones with at least min_cells cells; smaller ones (label fragments) are listed in `dropped`.
std::vector<Tunnel> loadTunnels(const std::string & file, int min_cells, std::vector<std::string> * dropped = nullptr);

}  // namespace nav_behaviors
