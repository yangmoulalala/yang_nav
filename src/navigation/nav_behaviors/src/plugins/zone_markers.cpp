#include "zone_markers.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include "nav_behaviors/zone_state.hpp"

namespace nav_behaviors
{

using Marker = visualization_msgs::msg::Marker;

MarkerArray zoneMarkers(const std::vector<Zone> & zones, const std::string & frame)
{
  MarkerArray markers;
  Marker clear;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);
  int id = 0;
  auto pt = [](const Pose2 & q) {
      geometry_msgs::msg::Point p;
      p.x = q.x;
      p.y = q.y;
      p.z = 0.02;
      return p;
    };
  for (const auto & t : zones) {
    const double hd = 0.5 * t.depth, hw = 0.5 * t.width;
    const bool open = t.profile == ZoneProfile::OpenDirectional;
    Marker box;
    box.header.frame_id = frame;
    box.ns = open ? "directional_body" : "narrow_body";
    box.id = id++;
    box.type = Marker::LINE_STRIP;
    box.scale.x = 0.02;
    box.color.r = 1.0f;
    box.color.g = open ? 1.0f : 0.6f;
    box.color.b = open ? 0.2f : 0.0f;
    box.color.a = 1.0f;
    box.pose.orientation.w = 1.0;
    for (const auto & uv : {std::pair{-hd, -hw}, {hd, -hw}, {hd, hw}, {-hd, hw}, {-hd, -hw}}) {
      box.points.push_back(pt(t.toMap(uv.first, uv.second, 0.0)));
    }
    markers.markers.push_back(box);
    // axis through the zone, 1.5 m beyond each mouth (where the shaper puts the path); a slope arrow
    // points uphill
    Marker ax = box;
    ax.ns = open ? "directional_axis" : "narrow_axis";
    ax.id = id++;
    ax.type = Marker::LINE_LIST;
    ax.scale.x = 0.01;
    ax.color.r = 0.2f;
    ax.color.g = 0.8f;
    ax.color.b = 1.0f;
    if (open) {
      ax.points = {pt(t.toMap(0.0, 0.0, 0.0)), pt(t.toMap(hd + 0.6, 0.0, 0.0))};
    } else {
      ax.points = {pt(t.toMap(-hd - 1.5, 0.0, 0.0)), pt(t.toMap(hd + 1.5, 0.0, 0.0))};
    }
    markers.markers.push_back(ax);
    Marker label = box;
    label.ns = open ? "directional_id" : "narrow_id";
    label.id = id++;
    label.type = Marker::TEXT_VIEW_FACING;
    label.points.clear();
    label.pose.position = pt(t.toMap(0.0, hw + 0.15, 0.0));
    label.pose.position.z = 0.3;
    label.scale.z = 0.15;
    label.color.r = label.color.g = label.color.b = 1.0f;
    label.text = t.id;
    markers.markers.push_back(label);
  }
  return markers;
}

MarkerArray trackerMarkers(
  const Pose2 & robot, const TrackerOutput & tracker, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  MarkerArray markers;
  Marker text;
  text.header.frame_id = frame;
  text.header.stamp = stamp;
  text.ns = "tracker_state";
  text.id = 0;
  text.type = Marker::TEXT_VIEW_FACING;
  text.action = tracker.weight > 0.0 ? Marker::ADD : Marker::DELETE;
  text.pose.position.x = robot.x;
  text.pose.position.y = robot.y + 0.55;
  text.pose.position.z = 0.5;
  text.pose.orientation.w = 1.0;
  text.scale.z = 0.12;
  text.color.a = 1.0f;
  text.color.r = 1.0f;
  text.color.g = tracker.gated ? 0.3f : 1.0f;
  text.color.b = tracker.gated ? 0.3f : 1.0f;
  char buf[200];
  std::snprintf(buf, sizeof(buf), "%s %s %s  w %.2f  s %+.2f\nlat %+.3f m  yaw %+.1f deg  v %.2f%s%s",
    tracker.zone_id.c_str(), zoneProfileName(tracker.profile), zonePhaseName(tracker.phase), tracker.weight, tracker.s_rel, tracker.e_lat,
    tracker.e_yaw * 180.0 / M_PI, tracker.v_ref, tracker.gated ? "  GATE" : "", tracker.backing_off ? "  BACK" : "");
  text.text = buf;
  markers.markers.push_back(text);
  Marker ref = text;
  ref.ns = "tracker_yaw_ref";
  ref.type = Marker::ARROW;
  ref.text.clear();
  ref.pose.position.x = robot.x;
  ref.pose.position.y = robot.y;
  ref.pose.position.z = 0.05;
  ref.pose.orientation.z = std::sin(0.5 * tracker.yaw_ref);
  ref.pose.orientation.w = std::cos(0.5 * tracker.yaw_ref);
  ref.scale.x = 0.5;
  ref.scale.y = 0.04;
  ref.scale.z = 0.04;
  ref.color.r = tracker.gated ? 1.0f : 0.1f;
  ref.color.g = tracker.gated ? 0.2f : 1.0f;
  ref.color.b = 0.2f;
  markers.markers.push_back(ref);
  return markers;
}

}  // namespace nav_behaviors
