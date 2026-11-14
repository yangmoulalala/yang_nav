#include <gtest/gtest.h>
#include "nav_behaviors/goal_pending.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_behaviors/zone_path_handoff.hpp"
using namespace nav_behaviors;  // NOLINT

TEST(PathHandoff, TemporaryLargeDetourWaitsButPersistentBlockageCanReroute)
{
  auto path = [](std::initializer_list<std::pair<double, double>> points) {
      nav_msgs::msg::Path p; p.header.frame_id = "map";
      for (const auto & xy : points) {
        geometry_msgs::msg::PoseStamped q;
        q.pose.position.x = xy.first; q.pose.position.y = xy.second;
        p.poses.push_back(q);
      }
      return p;
    };
  const auto current = path({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}});
  const auto long_route = path({{2, 0}, {2, 10}, {4, 10}, {4, 0}});
  const auto short_route = path({{2, 0}, {3, 0}, {4, 0}});
  DetourHold hold;
  EXPECT_TRUE(hold.keepCurrent(current, long_route, 10.));
  EXPECT_TRUE(hold.keepCurrent(current, long_route, 11.));
  EXPECT_FALSE(hold.keepCurrent(current, long_route, 11.6));
  // A short route reappearing after stale evidence expires resets the timer.
  EXPECT_FALSE(hold.keepCurrent(current, short_route, 12.));
  EXPECT_TRUE(hold.keepCurrent(current, long_route, 12.1));
  hold.reset();
  EXPECT_TRUE(hold.keepCurrent(current, long_route, 20.));
  EXPECT_FALSE(hold.keepCurrent(nav_msgs::msg::Path{}, long_route, 20.1));
  // A vehicle displaced from the retained path must immediately accept a fresh route.
  EXPECT_FALSE(hold.keepCurrent(current, path({{5, 0}, {5, 10}, {4, 0}}), 21.));
}

TEST(PathHandoff, NewGoalWaitsForItsOwnSuccessfulPlanAndShape)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CapturePathGoal>("CapturePathGoal");
  factory.registerNodeType<CommitZonePath>("CommitZonePath");
  factory.registerNodeType<AwaitGoalPath>("AwaitGoalPath");
  auto bb = BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal; goal.header.frame_id = "map"; goal.pose.orientation.w = 1.;
  bb->set("goal", goal);
  auto capture = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><CapturePathGoal signature="{planned}"/></BehaviorTree></root>)", bb);
  auto commit = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><CommitZonePath input_path="{shaped}" planned_signature="{planned}" path="{path}" signature="{ready}"/></BehaviorTree></root>)", bb);
  auto await = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><AwaitGoalPath signature="{ready}"/></BehaviorTree></root>)", bb);
  EXPECT_EQ(await.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(capture.tickOnce(), BT::NodeStatus::SUCCESS);
  nav_msgs::msg::Path path; path.poses.push_back(goal); bb->set("shaped", path);
  EXPECT_EQ(commit.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(await.tickOnce(), BT::NodeStatus::SUCCESS);
  goal.pose.position.x = 2.; bb->set("goal", goal);
  EXPECT_EQ(await.tickOnce(), BT::NodeStatus::RUNNING);  // rechecks even after SUCCESS
  // An old smoothing result arriving after preemption must not be bound to the new target.
  EXPECT_EQ(commit.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(await.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(capture.tickOnce(), BT::NodeStatus::SUCCESS);
  path.poses.back() = goal; bb->set("shaped", path);
  EXPECT_EQ(commit.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(await.tickOnce(), BT::NodeStatus::SUCCESS);
  // A distinct request for the same coordinates is still a new target.
  goal.header.stamp.sec = 1; bb->set("goal", goal);
  EXPECT_EQ(await.tickOnce(), BT::NodeStatus::RUNNING);
}

TEST(PathHandoff, DetourHoldRespectsRemappedOutputAndNewGoals)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CommitZonePath>("CommitZonePath");
  auto bb = BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map"; goal.pose.position.x = 4.; goal.pose.orientation.w = 1.;
  bb->set("goal", goal);
  nav_msgs::msg::Path short_path; short_path.header.frame_id = "map";
  for (double x : {0., 1., 2., 3., 4.}) {
    auto pose = goal; pose.pose.position.x = x; short_path.poses.push_back(pose);
  }
  bb->set("candidate", short_path);
  BT::NodeConfiguration config; config.blackboard = bb;
  bb->set("planned", targetSignature(config));
  auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><CommitZonePath input_path="{candidate}" planned_signature="{planned}" path="{executable}" signature="{ready}"/></BehaviorTree></root>)", bb);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  nav_msgs::msg::Path detour; detour.header.frame_id = "map";
  for (const auto & xy : {std::pair{2., 0.}, std::pair{2., 10.}, std::pair{4., 10.}, std::pair{4., 0.}}) {
    auto pose = goal; pose.pose.position.x = xy.first; pose.pose.position.y = xy.second;
    detour.poses.push_back(pose);
  }
  bb->set("candidate", detour);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("executable").poses.size(), short_path.poses.size());
  // A preempting goal, including a repeated coordinate request, bypasses the hold.
  goal.header.stamp.sec = 1; bb->set("goal", goal);
  bb->set("planned", targetSignature(config));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("executable").poses.size(), detour.poses.size());
}

TEST(PathHandoff, ApproachPointStaysPendingUntilOriginalGoalCanBeFollowed)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<KeepOccupiedGoalPending>("KeepOccupiedGoalPending");
  int follows=0;
  factory.registerSimpleAction("Follow",[&](BT::TreeNode &){++follows;return BT::NodeStatus::SUCCESS;});
  auto bb=BT::Blackboard::create();
  geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose.position.x=2.;
  bb->set("goal",goal);
  nav_msgs::msg::Path path;path.header.frame_id="map";path.poses.push_back(goal);path.poses.back().pose.position.x=1.;
  bb->set("path",path);
  auto tree=factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><KeepOccupiedGoalPending path="{path}"><Follow/></KeepOccupiedGoalPending></BehaviorTree></root>)",bb);
  EXPECT_EQ(tree.tickOnce(),BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(),BT::NodeStatus::RUNNING);EXPECT_EQ(follows,1);
  path.poses.back()=goal;bb->set("path",path);
  EXPECT_EQ(tree.tickOnce(),BT::NodeStatus::SUCCESS);EXPECT_EQ(follows,2);
  tree.haltTree();goal.pose.position.x=3.;bb->set("goal",goal);
  EXPECT_EQ(tree.tickOnce(),BT::NodeStatus::RUNNING);
}
