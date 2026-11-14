// Bind each executable path to the target that was successfully planned and shaped.
#pragma once
#include <iomanip>
#include <chrono>
#include <sstream>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav_behaviors/detour_hold.hpp"

namespace nav_behaviors
{
inline std::string targetSignature(const BT::NodeConfiguration & config)
{
  geometry_msgs::msg::PoseStamped goal;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (config.blackboard->get("goals", goals) && !goals.empty()) {goal = goals.back();}
  else if (!config.blackboard->get("goal", goal)) {return {};}
  std::ostringstream stream;
  const auto & p = goal.pose.position; const auto & q = goal.pose.orientation;
  stream << std::setprecision(17) << goal.header.frame_id << ' ' << goal.header.stamp.sec << ' '
    << goal.header.stamp.nanosec << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' '
    << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w;
  return stream.str();
}
class CapturePathGoal : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {BT::OutputPort<std::string>("signature")};}
  BT::NodeStatus tick() override {
    const auto sig = targetSignature(config());
    if (sig.empty()) {return BT::NodeStatus::FAILURE;}
    setOutput("signature", sig); return BT::NodeStatus::SUCCESS;
  }
};
class CommitZonePath : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("input_path"), BT::InputPort<std::string>("planned_signature"), BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<std::string>("signature")};
  }
  BT::NodeStatus tick() override {
    const auto path = getInput<nav_msgs::msg::Path>("input_path");
    const auto sig = targetSignature(config());
    const auto planned = getInput<std::string>("planned_signature");
    if (!planned || planned.value() != sig || !path || path.value().poses.empty() || sig.empty()) {return BT::NodeStatus::FAILURE;}
    if (sig == committed_goal_) {
      const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (detour_hold_.keepCurrent(committed_path_, path.value(), now)) {
        setOutput("path", committed_path_); setOutput("signature", sig);
        return BT::NodeStatus::SUCCESS;
      }
    } else {detour_hold_.reset();}
    setOutput("path", path.value()); setOutput("signature", sig);
    committed_goal_ = sig;
    committed_path_ = path.value();
    return BT::NodeStatus::SUCCESS;
  }
private:
  std::string committed_goal_;
  nav_msgs::msg::Path committed_path_;
  DetourHold detour_hold_;
};
class PathForGoal : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("signature")};}
  BT::NodeStatus tick() override {
    const auto sig = getInput<std::string>("signature"); const auto current = targetSignature(config());
    return sig && !current.empty() && sig.value() == current ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
class AwaitGoalPath : public BT::ActionNodeBase
{
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("signature")};}
  BT::NodeStatus tick() override {
    const auto sig = getInput<std::string>("signature"); const auto current = targetSignature(config());
    return sig && !current.empty() && sig.value() == current ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void halt() override {resetStatus();}
};
}  // namespace nav_behaviors
