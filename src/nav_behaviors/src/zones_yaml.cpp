#include "nav_behaviors/zones_yaml.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace nav_behaviors
{

ZoneLoad loadZones(
  const std::string & file, int min_cells, int slope_min_cells, const std::vector<std::string> & types)
{
  ZoneLoad out;
  const std::set<std::string> wanted(types.begin(), types.end());
  const YAML::Node root = YAML::LoadFile(file);
  for (const auto & z : root["zones"]) {
    const std::string type = z["type"] ? z["type"].as<std::string>() : std::string();
    if (wanted.count(type) == 0) {
      continue;
    }
    // Every exported directional terrain is a passage zone. The semantic label selects a
    // default control profile; explicit traversal fields in zones.yaml define the geometry policy.
    const bool tunnel = type == "tunnel";
    const bool open_directional = !tunnel;
    const auto id = z["id"] ? z["id"].as<std::string>() : std::string("zone");
    const int cells = z["cells"] ? z["cells"].as<int>() : 0;
    const int need = open_directional ? slope_min_cells : min_cells;
    if (cells < need) {
      out.dropped.push_back(id + " (" + std::to_string(cells) + " cells)");
      continue;
    }
    Zone t;
    t.id = id;
    t.profile = tunnel ? ZoneProfile::NarrowPassage : ZoneProfile::OpenDirectional;
    t.cx = z["centre"][0].as<double>();
    t.cy = z["centre"][1].as<double>();
    t.yaw = z["yaw_deg"].as<double>() * M_PI / 180.0;
    t.depth = z["size"][0].as<double>();
    t.width = z["size"][1].as<double>();
    // Backward-compatible defaults for old terrain packages. Tunnels (and undulations)
    // follow a bidirectional axis. Slopes and steps face the annotated positive direction.
    const bool bidirectional_default = tunnel || type == "undulation";
    t.one_way_dir = z["one_way_dir"] ? z["one_way_dir"].as<int>() : (bidirectional_default ? 0 : 1);
    if (t.one_way_dir < -1 || t.one_way_dir > 1) {
      throw std::runtime_error("zone " + id + ": one_way_dir must be -1, 0, or 1");
    }
    t.allow_flip = z["allow_flip"] ? z["allow_flip"].as<bool>() : (t.one_way_dir == 0);
    t.recentre = z["recentre"] ? z["recentre"].as<bool>() : tunnel;
    if (z["warning"]) {
      const std::string w = z["warning"].as<std::string>();
      if (w.find("direction varies") != std::string::npos) {
        t.axis_reliable = false;
        ++out.ambiguous;
      }
    }
    out.zones.push_back(t);
    (tunnel ? out.tunnels : out.directional) += 1;
  }
  return out;
}

std::vector<Zone> loadTunnels(const std::string & file, int min_cells, std::vector<std::string> * dropped)
{
  const ZoneLoad l = loadZones(file, min_cells, min_cells, {"tunnel"});
  if (dropped) {
    *dropped = l.dropped;
  }
  return l.zones;
}

}  // namespace nav_behaviors
