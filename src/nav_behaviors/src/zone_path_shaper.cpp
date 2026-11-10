// ZonePathShaper: nav2_core::Smoother plugin around nav_behaviors::shapePath.
//
// Parameters (<name>.):
//   zones_file         zones.yaml of the terrain package (set by navigation.launch.py)
//   min_cells          tunnel zones smaller than this are label fragments and ignored (50)
//   axis_margin, lead_length, lead_growth, max_retries, spacing   see path_shaper.hpp
//   footprint, footprint_padding   robot outline for the collision check of the shaped stretch
// The collision check uses the smoother_server costmap (global costmap) and rejects only lethal /
// unknown cells: inside a 0.8 m tunnel the inscribed band covers the whole opening.
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav2_core/smoother.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/zones_yaml.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav_behaviors
{

class ZonePathShaper : public nav2_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    costmap_sub_ = costmap_sub;
    using nav2_util::declare_parameter_if_not_declared;
    using rclcpp::ParameterValue;
    const std::string p = name + ".";
    declare_parameter_if_not_declared(node, p + "zones_file", ParameterValue(std::string()));
    declare_parameter_if_not_declared(node, p + "min_cells", ParameterValue(50));
    declare_parameter_if_not_declared(node, p + "axis_margin", ParameterValue(params_.axis_margin));
    declare_parameter_if_not_declared(node, p + "lead_length", ParameterValue(params_.lead_length));
    declare_parameter_if_not_declared(node, p + "lead_growth", ParameterValue(params_.lead_growth));
    declare_parameter_if_not_declared(node, p + "max_retries", ParameterValue(params_.max_retries));
    declare_parameter_if_not_declared(node, p + "spacing", ParameterValue(params_.spacing));
    declare_parameter_if_not_declared(node, p + "max_body_offset", ParameterValue(params_.max_body_offset));
    declare_parameter_if_not_declared(node, p + "max_side_offset", ParameterValue(params_.max_side_offset));
    declare_parameter_if_not_declared(node, p + "offset_step", ParameterValue(params_.offset_step));
    declare_parameter_if_not_declared(node, p + "tangent_window", ParameterValue(params_.tangent_window));
    declare_parameter_if_not_declared(node, p + "footprint", ParameterValue(
        std::string("[[0.38, 0.30], [0.38, -0.30], [-0.30, -0.30], [-0.30, 0.30]]")));
    declare_parameter_if_not_declared(node, p + "footprint_padding", ParameterValue(0.01));
    // shaped stretch is checked with the footprint shrunk by this much: a few cm against the map are left to
    // the local controller (MPPI checks the real footprint)
    declare_parameter_if_not_declared(node, p + "collision_tolerance", ParameterValue(0.04));
    std::string zones_file, fp;
    int min_cells = 50;
    double padding = 0.0;
    node->get_parameter(p + "zones_file", zones_file);
    node->get_parameter(p + "min_cells", min_cells);
    node->get_parameter(p + "axis_margin", params_.axis_margin);
    node->get_parameter(p + "lead_length", params_.lead_length);
    node->get_parameter(p + "lead_growth", params_.lead_growth);
    node->get_parameter(p + "max_retries", params_.max_retries);
    node->get_parameter(p + "spacing", params_.spacing);
    node->get_parameter(p + "max_body_offset", params_.max_body_offset);
    node->get_parameter(p + "max_side_offset", params_.max_side_offset);
    node->get_parameter(p + "offset_step", params_.offset_step);
    node->get_parameter(p + "tangent_window", params_.tangent_window);
    node->get_parameter(p + "footprint", fp);
    node->get_parameter(p + "footprint_padding", padding);
    double tolerance = 0.04;
    node->get_parameter(p + "collision_tolerance", tolerance);
    if (!nav2_costmap_2d::makeFootprintFromString(fp, footprint_)) {
      throw std::runtime_error("ZonePathShaper: invalid footprint " + fp);
    }
    nav2_costmap_2d::padFootprint(footprint_, padding - tolerance);
    loadZones(zones_file, min_cells);
  }

  void cleanup() override
  {
    costmap_sub_.reset();
    tunnels_.clear();
  }
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration &) override
  {
    if (tunnels_.empty() || path.poses.size() < 3) {
      return true;
    }
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
    try {
      costmap = costmap_sub_->getCostmap();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "ZonePathShaper: no costmap (%s), path kept", e.what());
      return true;
    }
    nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>> checker(costmap);
    const double lethal = static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    auto collision_free = [&](const Pose2 & q) {
        return checker.footprintCostAtPose(q.x, q.y, q.yaw, footprint_) < lethal;
      };

    std::vector<Pose2> in;
    in.reserve(path.poses.size());
    for (const auto & ps : path.poses) {
      const auto & o = ps.pose.orientation;
      in.push_back({ps.pose.position.x, ps.pose.position.y,
          std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))});
    }
    std::vector<ShapeEvent> events;
    const auto out = shapePath(in, tunnels_, params_, collision_free, &events);
    for (const auto & e : events) {
      if (e.shaped) {
        RCLCPP_INFO(logger_, "ZonePathShaper: %s shaped, dir %+d, heading %.1f deg, lead %.2f m, turn %s, "
          "offset in/body/out %+.2f/%+.2f/%+.2f m", e.tunnel_id.c_str(), e.direction, e.heading * 180.0 / M_PI,
          e.lead, turnName(e.turn), e.v_in, e.v_body, e.v_out);
      } else {
        RCLCPP_WARN(logger_, "ZonePathShaper: %s planner path kept (%s)", e.tunnel_id.c_str(),
          e.reason.c_str());
      }
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
  void loadZones(const std::string & file, int min_cells)
  {
    tunnels_.clear();
    if (file.empty()) {
      RCLCPP_WARN(logger_, "ZonePathShaper: zones_file not set, paths pass through unchanged");
      return;
    }
    std::vector<std::string> dropped;
    tunnels_ = loadTunnels(file, min_cells, &dropped);
    for (const auto & d : dropped) {
      RCLCPP_WARN(logger_, "ZonePathShaper: dropping %s, label fragment", d.c_str());
    }
    RCLCPP_INFO(logger_, "ZonePathShaper: %zu tunnels from %s", tunnels_.size(), file.c_str());
  }

  rclcpp::Logger logger_{rclcpp::get_logger("ZonePathShaper")};
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::vector<Tunnel> tunnels_;
  ShaperParams params_;
};

}  // namespace nav_behaviors

PLUGINLIB_EXPORT_CLASS(nav_behaviors::ZonePathShaper, nav2_core::Smoother)
