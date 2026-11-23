// A safe approach point must never complete the original occupied target request.
#pragma once
#include <cmath>
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "nav_behaviors/zone_path_handoff.hpp"
namespace nav_behaviors {
inline bool pathEndsAtRequestedGoal(const BT::NodeConfiguration & config,
  const nav_msgs::msg::Path & path) {
  geometry_msgs::msg::PoseStamped goal;
  if (path.poses.empty() || !config.blackboard->get("goal", goal)) {return false;}
  if (goal.header.frame_id != path.header.frame_id) {
    std::shared_ptr<tf2_ros::Buffer> tf;
    if (!config.blackboard->get("tf_buffer",tf) || !tf) {return false;}
    try {goal=tf->transform(goal,path.header.frame_id,tf2::durationFromSec(0.0));}
    catch (const tf2::TransformException &) {return false;}
  }
  const auto & end = path.poses.back().pose.position;
  return std::hypot(end.x-goal.pose.position.x, end.y-goal.pose.position.y) <= 0.20;
}
class PathReachesGoal : public BT::ConditionNode {
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override {
    auto path=getInput<nav_msgs::msg::Path>("path");
    return path && pathEndsAtRequestedGoal(config(),path.value()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
class KeepOccupiedGoalPending : public BT::DecoratorNode {
public:
  using BT::DecoratorNode::DecoratorNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override {
    auto input=getInput<nav_msgs::msg::Path>("path");
    if (!input || input.value().poses.empty()) {return BT::NodeStatus::FAILURE;}
    const auto & path=input.value();const auto & end=path.poses.back().pose.position;
    const auto sig=targetSignature(config());
    const bool complete=pathEndsAtRequestedGoal(config(),path);
    if (sig!=signature_ || complete!=complete_ || std::hypot(end.x-x_,end.y-y_)>.05) {
      resetChild();waiting_=false;signature_=sig;x_=end.x;y_=end.y;complete_=complete;
    }
    setStatus(BT::NodeStatus::RUNNING);
    if(waiting_) {return BT::NodeStatus::RUNNING;}
    auto status=child_node_->executeTick();
    if(status==BT::NodeStatus::SUCCESS && !complete) {
      resetChild();waiting_=true;return BT::NodeStatus::RUNNING;
    }
    return status;
  }
  void halt() override {waiting_=false;signature_.clear();BT::DecoratorNode::halt();}
private:
  bool waiting_{false},complete_{false};double x_{0},y_{0};std::string signature_;
};
}  // namespace nav_behaviors