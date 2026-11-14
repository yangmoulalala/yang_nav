// ZonePhaseMonitor: the /nav_zone_state subscription shared by the behaviour-tree nodes.
//
// The controller latches "traverse tunnel_3 @123.4" (transient local, re-published every 0.5 s).  A BT
// node is ticked only now and then, so it must drain its own callback group and judge freshness by the
// stamp inside the message, not by the time it happens to process it: a latched sample of an aborted
// run then never looks fresh.
//
//   fresh(t)           the newest phase is younger than t
//   paused(t)          fresh and the newest phase is one that pauses replanning
//   blockedRecently(t) ALIGN / TRAVERSE / EXIT was reported within t (no spin, no back-up there)
#pragma once

#include <memory>
#include <string>

#include "nav_behaviors/zone_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace nav_behaviors
{

class ZonePhaseMonitor
{
public:
  // Idempotent: re-subscribes only when the topic changed.
  void initialize(const rclcpp::Node::SharedPtr & node, const std::string & topic);
  // Drain the callback group; call once per tick.
  void spin();

  bool ready() const {return static_cast<bool>(sub_);}
  const std::string & topic() const {return topic_;}

  bool fresh(double timeout) const;
  bool paused(double timeout) const;
  bool blockedRecently(double timeout) const;
  bool pathBlocked(double timeout) const {return fresh(timeout) && memory_.path_blocked;}

private:
  double now() const;
  void onPhase(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string topic_;
  PhaseMemory memory_;
};

}  // namespace nav_behaviors
