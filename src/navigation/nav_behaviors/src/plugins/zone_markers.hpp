#pragma once

#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_behaviors/zone_tracker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav_behaviors
{

using MarkerArray = visualization_msgs::msg::MarkerArray;
MarkerArray zoneMarkers(const std::vector<Zone> & zones, const std::string & frame);
MarkerArray trackerMarkers(
  const Pose2 & robot, const TrackerOutput & tracker, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp);

}  // namespace nav_behaviors
