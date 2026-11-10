#include "nav_behaviors/zones_yaml.hpp"

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace nav_behaviors
{

std::vector<Tunnel> loadTunnels(const std::string & file, int min_cells, std::vector<std::string> * dropped)
{
  std::vector<Tunnel> out;
  const YAML::Node root = YAML::LoadFile(file);
  for (const auto & z : root["zones"]) {
    if (z["type"].as<std::string>() != "tunnel") {
      continue;
    }
    const int cells = z["cells"] ? z["cells"].as<int>() : 0;
    const auto id = z["id"].as<std::string>();
    if (cells < min_cells) {
      if (dropped) {
        dropped->push_back(id + " (" + std::to_string(cells) + " cells)");
      }
      continue;
    }
    Tunnel t;
    t.id = id;
    t.cx = z["centre"][0].as<double>();
    t.cy = z["centre"][1].as<double>();
    t.yaw = z["yaw_deg"].as<double>() * M_PI / 180.0;
    t.depth = z["size"][0].as<double>();
    t.width = z["size"][1].as<double>();
    out.push_back(t);
  }
  return out;
}

}  // namespace nav_behaviors
