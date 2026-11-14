// ZonePathShaper: nav2_core::Smoother plugin around nav_behaviors::shapePath (direction-constrained terrain).
//
// Parameters (<name>.):
//   zones_file         zones.yaml of the terrain package (set by navigation.launch.py)
//   min_cells          tunnel zones smaller than this are label fragments and ignored (50)
//   slope_min_cells    ... and slope zones (50)
//   types              directional terrain types to shape (see nav2.yaml)
//   axis_margin, lead_length, lead_growth, max_retries, spacing   see path_shaper.hpp
//   slope_lead_length, slope_axis_margin, slope_max_body_offset   ... for slopes
//   chain_gap, chain_angle_deg, chain_lat   zones shaped as one stretch (tunnel_3+slope_3 ...)
//   footprint, footprint_padding   complete robot outline and padding for collision checks
//   collision_tolerance  compatibility parameter; only zero is accepted
// The collision check uses the smoother_server costmap (global costmap) and rejects only lethal /
// unknown cells: inside a 0.8 m tunnel the inscribed band covers the whole opening.
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_core/smoother_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/costmap_collision.hpp"
#include "nav_behaviors/overlap_escape.hpp"
#include "nav_behaviors/zones_yaml.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"

namespace nav_behaviors
{

class ZonePathShaper : public nav2_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    costmap_sub_ = costmap_sub;
    tf_ = tf;
    using nav2_util::declare_parameter_if_not_declared;
    using rclcpp::ParameterValue;
    const std::string p = name + ".";
    declare_parameter_if_not_declared(node, p + "robot_base_frame", ParameterValue(std::string("base_link")));
    node->get_parameter(p + "robot_base_frame", base_frame_);
    declare_parameter_if_not_declared(node, p + "zones_file", ParameterValue(std::string()));
    declare_parameter_if_not_declared(node, p + "min_cells", ParameterValue(50));
    declare_parameter_if_not_declared(node, p + "slope_min_cells", ParameterValue(50));
    const std::vector<std::string> default_zone_types{
      "tunnel", "slope", "step_l1", "step_l2", "step_high", "fly_slope", "undulation"};
    declare_parameter_if_not_declared(node, p + "types", ParameterValue(default_zone_types));
    declare_parameter_if_not_declared(node, p + "axis_margin", ParameterValue(params_.axis_margin));
    declare_parameter_if_not_declared(node, p + "lead_length", ParameterValue(params_.lead_length));
    declare_parameter_if_not_declared(node, p + "lead_growth", ParameterValue(params_.lead_growth));
    declare_parameter_if_not_declared(node, p + "max_retries", ParameterValue(params_.max_retries));
    declare_parameter_if_not_declared(node, p + "spacing", ParameterValue(params_.spacing));
    declare_parameter_if_not_declared(node, p + "max_body_offset", ParameterValue(params_.max_body_offset));
    declare_parameter_if_not_declared(node, p + "max_side_offset", ParameterValue(params_.max_side_offset));
    declare_parameter_if_not_declared(node, p + "offset_step", ParameterValue(params_.offset_step));
    declare_parameter_if_not_declared(node, p + "tangent_window", ParameterValue(params_.tangent_window));
    declare_parameter_if_not_declared(node, p + "early_turn", ParameterValue(params_.early_turn));
    declare_parameter_if_not_declared(node, p + "early_turn_max_yaw", ParameterValue(params_.early_turn_max_yaw));
    declare_parameter_if_not_declared(node, p + "slope_lead_length", ParameterValue(params_.slope_lead_length));
    declare_parameter_if_not_declared(node, p + "slope_axis_margin", ParameterValue(params_.slope_axis_margin));
    declare_parameter_if_not_declared(node, p + "slope_max_body_offset",
      ParameterValue(params_.slope_max_body_offset));
    declare_parameter_if_not_declared(node, p + "chain_gap", ParameterValue(params_.chain.gap));
    declare_parameter_if_not_declared(node, p + "chain_angle_deg",
      ParameterValue(params_.chain.angle * 180.0 / M_PI));
    declare_parameter_if_not_declared(node, p + "chain_lat", ParameterValue(params_.chain.lat));
    node->get_parameter(p + "early_turn", params_.early_turn);
    node->get_parameter(p + "early_turn_max_yaw", params_.early_turn_max_yaw);
    declare_parameter_if_not_declared(node, p + "footprint", ParameterValue(
        std::string("[[0.38, 0.30], [0.38, -0.30], [-0.30, -0.30], [-0.30, 0.30]]")));
    declare_parameter_if_not_declared(node, p + "footprint_padding", ParameterValue(0.01));
    // Retained for configuration compatibility; nonzero values are rejected below.
    // Both shaping and tracking must check the complete padded footprint.
    declare_parameter_if_not_declared(node, p + "collision_tolerance", ParameterValue(0.0));
    declare_parameter_if_not_declared(node, p + "escape_max_distance", ParameterValue(1.5));
    node->get_parameter(p + "escape_max_distance", escape_max_distance_);
    if (escape_max_distance_ <= 0.0 || escape_max_distance_ > 2.0) {
      throw std::runtime_error("ZonePathShaper: invalid escape_max_distance");
    }
    std::string zones_file, fp;
    int min_cells = 50, slope_min_cells = 50;
    double padding = 0.0, chain_angle_deg = params_.chain.angle * 180.0 / M_PI;
    std::vector<std::string> types = default_zone_types;
    node->get_parameter(p + "zones_file", zones_file);
    node->get_parameter(p + "min_cells", min_cells);
    node->get_parameter(p + "slope_min_cells", slope_min_cells);
    node->get_parameter(p + "types", types);
    node->get_parameter(p + "axis_margin", params_.axis_margin);
    node->get_parameter(p + "lead_length", params_.lead_length);
    node->get_parameter(p + "lead_growth", params_.lead_growth);
    node->get_parameter(p + "max_retries", params_.max_retries);
    node->get_parameter(p + "spacing", params_.spacing);
    node->get_parameter(p + "max_body_offset", params_.max_body_offset);
    node->get_parameter(p + "max_side_offset", params_.max_side_offset);
    node->get_parameter(p + "offset_step", params_.offset_step);
    node->get_parameter(p + "tangent_window", params_.tangent_window);
    node->get_parameter(p + "slope_lead_length", params_.slope_lead_length);
    node->get_parameter(p + "slope_axis_margin", params_.slope_axis_margin);
    node->get_parameter(p + "slope_max_body_offset", params_.slope_max_body_offset);
    node->get_parameter(p + "chain_gap", params_.chain.gap);
    node->get_parameter(p + "chain_angle_deg", chain_angle_deg);
    node->get_parameter(p + "chain_lat", params_.chain.lat);
    params_.chain.angle = chain_angle_deg * M_PI / 180.0;
    node->get_parameter(p + "footprint", fp);
    node->get_parameter(p + "footprint_padding", padding);
    double tolerance = 0.0;
    node->get_parameter(p + "collision_tolerance", tolerance);
    if (!nav2_costmap_2d::makeFootprintFromString(fp, footprint_)) {
      throw std::runtime_error("ZonePathShaper: invalid footprint " + fp);
    }
    if (tolerance != 0.0) {
      throw std::runtime_error("ZonePathShaper: collision_tolerance must be zero; shrinking the physical footprint is unsafe");
    }
    nav2_costmap_2d::padFootprint(footprint_, padding);
    for (const auto & point : footprint_) {
      params_.exit_clearance = std::max(params_.exit_clearance, std::abs(point.x) + .05);
    }
    if (params_.spacing <= 0.0 || params_.lead_length < 0.0 || params_.max_retries < 0) {
      throw std::runtime_error("ZonePathShaper: invalid spacing, lead_length or max_retries");
    }
    loadZones(zones_file, min_cells, slope_min_cells, types);
  }

  void cleanup() override
  {
    costmap_sub_.reset();
    zones_.clear();
  }
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration &) override
  {
    if (zones_.empty()) {
      return true;
    }
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    try {
      costmap = costmap_sub_->getCostmap();
    } catch (const std::exception & e) {
      throw nav2_core::FailedToSmoothPath(std::string("ZonePathShaper: no costmap: ") + e.what());
    }
    if (path.poses.empty()) {throw nav2_core::InvalidPath("ZonePathShaper: path is empty");}
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    auto collision_free = [&](const Pose2 & q) {
        return footprintFree(*costmap, footprint_, q);
      };

    std::vector<Pose2> in;
    in.reserve(path.poses.size());
    for (const auto & ps : path.poses) {
      const auto & o = ps.pose.orientation;
      in.push_back({ps.pose.position.x, ps.pose.position.y,
          std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))});
    }
    // Coarse plans must not jump over an entire small zone between two samples.
    std::vector<Pose2> dense{in.front()};
    for (size_t i = 1; i < in.size(); ++i) {
      const auto & a = in[i - 1];
      const auto & b = in[i];
      const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / params_.spacing)));
      for (int j = 1; j <= count; ++j) {
        const double f = static_cast<double>(j) / count;
        dense.push_back({a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
          normAngle(a.yaw + f * normAngle(b.yaw - a.yaw))});
      }
    }
    in.swap(dense);
    // Preserve a validated translation escape before shaping the collision-free suffix.
    // The exception is restricted to a non-growing overlap with the original component
    // in this prefix, never the whole path.
    std::vector<Pose2> escape_prefix;
    if (!collision_free(in.front())) {
      const auto first = in.front();
      const auto allowed = escapeAllowance(*costmap, footprintContacts(*costmap, footprint_, first), first, escape_max_distance_);
      size_t free_index = 0;
      while (free_index < in.size() && !collision_free(in[free_index])) {++free_index;}
      if (free_index == in.size()) {
        throw nav2_core::SmoothedPathInCollision("ZonePathShaper: occupied path has no free escape endpoint");
      }
      for (size_t i = 1; i <= free_index; ++i) {
        if (std::hypot(in[i].x - first.x, in[i].y - first.y) > escape_max_distance_ + 1e-6 ||
          !escapeMotionFree(*costmap, footprint_, in[i - 1], in[i], &allowed)) {
          throw nav2_core::SmoothedPathInCollision("ZonePathShaper: escape adds contacts or changes heading");
        }
      }
      escape_prefix.assign(in.begin(), in.begin() + free_index);
      in.erase(in.begin(), in.begin() + free_index);
    }
    std::vector<ShapeEvent> events;
    // robot heading, when the path starts at the robot (ComputePathToPose plans from the robot pose): the
    // lead-in turns from there instead of from the grid tangent
    double robot_yaw = in.front().yaw;
    bool have_robot = !escape_prefix.empty();
    try {
      const auto tfm = tf_->lookupTransform(path.header.frame_id, base_frame_, tf2::TimePointZero);
      const auto & r = tfm.transform.rotation;
      const double dx = tfm.transform.translation.x - in.front().x, dy = tfm.transform.translation.y - in.front().y;
      if (std::hypot(dx, dy) < 0.3) {
        robot_yaw = std::atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z));
        have_robot = true;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "ZonePathShaper: no robot pose (%s), lead-in from the "
        "planner tangent", e.what());
    }
    auto out = shapePath(in, zones_, params_, collision_free, &events, have_robot ? &robot_yaw : nullptr);
    if (in.size() == 1 && !collision_free(in.front())) {
      throw nav2_core::SmoothedPathInCollision("ZonePathShaper: single-pose goal is occupied");
    }
    for (const auto & e : events) {
      if (e.shaped) {
        RCLCPP_INFO(logger_, "ZonePathShaper: %s (%s%s) shaped, dir %+d, heading %.1f deg, lead %.2f m, turn %s, "
          "offset in/body/out %+.2f/%+.2f/%+.2f m", e.zone_id.c_str(), zoneProfileName(e.profile),
          e.chain > 1 ? " chain" : "", e.direction, e.heading * 180.0 / M_PI, e.lead, turnName(e.turn),
          e.v_in, e.v_body, e.v_out);
      } else {
        RCLCPP_WARN(logger_, "ZonePathShaper: %s planner path kept (%s)", e.zone_id.c_str(), e.reason.c_str());
        if ((e.profile == ZoneProfile::NarrowPassage || e.direction != 0) && e.reason.find("downhill") == std::string::npos &&
          e.reason.find("ambiguous") == std::string::npos) {
          throw nav2_core::FailedToSmoothPath("ZonePathShaper: " + e.zone_id + ": " + e.reason);
        }
      }
    }
    if (std::any_of(events.begin(), events.end(), [](const ShapeEvent & e) {return e.shaped;})) {
      for (size_t i = 1; i < out.size(); ++i) {
        Pose2 hit;
        if (!footprintMotionFree(*costmap, footprint_, out[i - 1], out[i], &hit)) {
          throw nav2_core::SmoothedPathInCollision("ZonePathShaper: swept footprint intersects occupied space at (" +
            std::to_string(hit.x) + ", " + std::to_string(hit.y) + ", yaw " + std::to_string(hit.yaw) +
            "), segment " + std::to_string(i) + " from yaw " + std::to_string(out[i - 1].yaw) +
            " to " + std::to_string(out[i].yaw));
        }
      }
    }
    if (!escape_prefix.empty()) {
      // Shaping must preserve the free handoff pose, including heading, so it cannot
      // rotate the overlapping tail of the escape into a wall.
      if (out.empty() || std::hypot(out.front().x - in.front().x, out.front().y - in.front().y) > 1e-6 ||
        std::abs(normAngle(out.front().yaw - in.front().yaw)) > 1e-3) {
        throw nav2_core::SmoothedPathInCollision("ZonePathShaper: shaping changed the escape handoff");
      }
      out.insert(out.begin(), escape_prefix.begin(), escape_prefix.end());
    }
    const auto header = path.poses.front().header;
    path.poses.clear();
    path.poses.reserve(out.size());
    for (const auto & q : out) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = q.x;
      ps.pose.position.y = q.y;
      ps.pose.orientation.z = std::sin(q.yaw / 2.0);
      ps.pose.orientation.w = std::cos(q.yaw / 2.0);
      path.poses.push_back(ps);
    }
    return true;
  }

private:
  void loadZones(const std::string & file, int min_cells, int slope_min_cells,
    const std::vector<std::string> & types)
  {
    zones_.clear();
    if (file.empty()) {
      RCLCPP_WARN(logger_, "ZonePathShaper: zones_file not set, paths pass through unchanged");
      return;
    }
    const ZoneLoad l = nav_behaviors::loadZones(file, min_cells, slope_min_cells, types);
    zones_ = l.zones;
    for (const auto & d : l.dropped) {
      RCLCPP_WARN(logger_, "ZonePathShaper: dropping %s, label fragment", d.c_str());
    }
    RCLCPP_INFO(logger_, "ZonePathShaper: %d narrow zones, %d directional zones from %s (%d ambiguous direction), %zu stretches "
      "after chaining", l.tunnels, l.directional, file.c_str(), l.ambiguous, groupZones(zones_, params_.chain).size());
  }

  rclcpp::Logger logger_{rclcpp::get_logger("ZonePathShaper")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string base_frame_{"base_link"};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::vector<Zone> zones_;
  ShaperParams params_;
  double escape_max_distance_{1.5};
};

}  // namespace nav_behaviors

PLUGINLIB_EXPORT_CLASS(nav_behaviors::ZonePathShaper, nav2_core::Smoother)
