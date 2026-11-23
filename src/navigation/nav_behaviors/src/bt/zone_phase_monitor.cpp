#include "nav_behaviors/zone_phase_monitor.hpp"

namespace nav_behaviors
{

void ZonePhaseMonitor::initialize(const rclcpp::Node::SharedPtr & node, const std::string & topic)
{
  if (sub_ && topic == topic_) {
    return;
  }
  node_ = node;
  topic_ = topic;
  sub_.reset();
  if (group_) {executor_.remove_callback_group(group_);}
  memory_ = PhaseMemory{};
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = group_;
  // deep queue: a BT node is ticked only now and then (recovery), every phase change must still be seen
  sub_ = node_->create_subscription<std_msgs::msg::String>(
    topic_, rclcpp::QoS(100).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {onPhase(msg);}, opt);
}

void ZonePhaseMonitor::spin()
{
  if (group_) {
    executor_.spin_some();
  }
}

void ZonePhaseMonitor::onPhase(const std_msgs::msg::String::SharedPtr msg)
{
  double stamp = 0.0;
  const ZonePhase p = parseZonePhase(msg->data, &stamp);
  if (stamp > now() + 0.01) {return;}  // future samples cannot freeze a new clock epoch
  memory_.update(p, stamp);
}

double ZonePhaseMonitor::now() const
{
  return node_ ? node_->now().seconds() : 0.0;
}

bool ZonePhaseMonitor::fresh(double timeout) const
{
  return memory_.fresh(now(), timeout);
}

bool ZonePhaseMonitor::paused(double timeout) const
{
  return memory_.pausedNow(now(), timeout);
}

bool ZonePhaseMonitor::blockedRecently(double timeout) const
{
  return memory_.blockedRecently(now(), timeout);
}

}  // namespace nav_behaviors
