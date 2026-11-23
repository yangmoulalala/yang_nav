// ZoneReplanPaused (BT condition): SUCCESS while the ZoneAwareController is inside a zone stretch
// (align / traverse / exit), FAILURE otherwise.
//
// The behaviour tree keeps the current path while this returns SUCCESS: the shaped path is the only
// one that fits the opening, and replacing it half way through a tunnel (path expired, IsPathValid
// false) would interrupt zone alignment.  The phase arrives on /nav_zone_state ("traverse tunnel_3 @<stamp s>",
// published by the controller with its clock).  Freshness uses that stamp, not the time the BT happens
// to process the message, so a latched sample of an aborted run never looks fresh.
//
//   timeout     the newest phase must be younger than this (s); missing or stale -> FAILURE
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav_behaviors/zone_phase_monitor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav_behaviors
{

class ZoneReplanPaused : public BT::ConditionNode
{
public:
  ZoneReplanPaused(const std::string & name, const BT::NodeConfiguration & conf);
  ZoneReplanPaused() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("state_topic", "/nav_zone_state", "zone phase topic"),
      BT::InputPort<double>("timeout", 1.5, "no message for this long -> not paused"),
    };
  }

protected:
  void initialize();

  ZonePhaseMonitor monitor_;
  double timeout_{1.5};
};

// A blocked footprint must invalidate even a point-wise valid path in a zone.
class ZonePathBlocked : public ZoneReplanPaused
{
public:
  using ZoneReplanPaused::ZoneReplanPaused;
  BT::NodeStatus tick() override {
    if (!BT::isStatusActive(status()) || !monitor_.ready()) {initialize();}
    monitor_.spin();
    return monitor_.pathBlocked(timeout_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace nav_behaviors
