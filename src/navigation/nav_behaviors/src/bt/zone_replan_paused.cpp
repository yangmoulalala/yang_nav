// ZoneReplanPaused: BT condition around nav_behaviors::phasePausesReplan.
#include <memory>
#include <string>

#include "nav_behaviors/zone_phase_monitor.hpp"
#include "nav_behaviors/zone_replan_paused.hpp"

namespace nav_behaviors
{

ZoneReplanPaused::ZoneReplanPaused(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
}

void ZoneReplanPaused::initialize()
{
  std::string topic = "/nav_zone_state";
  getInput("state_topic", topic);
  getInput("timeout", timeout_);
  monitor_.initialize(config().blackboard->get<rclcpp::Node::SharedPtr>("node"), topic);
}

BT::NodeStatus ZoneReplanPaused::tick()
{
  if (!BT::isStatusActive(status()) || !monitor_.ready()) {
    initialize();
  }
  monitor_.spin();
  return monitor_.paused(timeout_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace nav_behaviors
