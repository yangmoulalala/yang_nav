// ZoneRecovery behaviour-tree test: the ladder really escalates inside a tree, stops escalating
// when the navigation child succeeds, resets between goals, and never moves the robot inside a zone.
//
// The zone state is driven the way the controller drives it: a stamped /nav_zone_state message on a
// real (non-sim) node in the blackboard, so the freshness rules of PhaseMemory are exercised too.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <filesystem>
#include <fstream>

#include "behaviortree_cpp/bt_factory.h"
#include "nav_behaviors/zone_recovery.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace nav_behaviors;  // NOLINT

namespace
{

// The BT stubs get no access to config() (protected), so the current blackboard is shared through
// this pointer instead. One tree runs at a time in this test.
BT::Blackboard::Ptr g_bb;

// One rclcpp context for the whole binary: creating and destroying it per test crashed in the DDS
// teardown of the node (rmw_publish during delete datareader).
class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

const bool g_ros_environment = []() {
    ::testing::AddGlobalTestEnvironment(new RosEnvironment);
    return true;
  }();

struct Log
{
  std::vector<std::string> actions;   // recovery children, in tick order
  int nav_failures{0};
  int nav_ticks{0};
};

using LogPtr = std::shared_ptr<Log>;

LogPtr logOf()
{
  return g_bb->get<LogPtr>("log");
}

// navigation subtree stand-in: fails while `nav_fails` is positive, then succeeds
BT::NodeStatus failingNav(BT::TreeNode &)
{
  auto log = logOf();
  ++log->nav_ticks;
  int fails = 0;
  if (!g_bb->get("nav_fails", fails)) {
    return BT::NodeStatus::FAILURE;
  }
  if (fails > 0) {
    g_bb->set("nav_fails", fails - 1);
    ++log->nav_failures;
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::SimpleActionNode::TickFunctor record(const std::string & name)
{
  return [name](BT::TreeNode &) {
           logOf()->actions.push_back(name);
           return BT::NodeStatus::SUCCESS;
         };
}

const char * kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ZoneRecovery retries="4" state_topic="/nav_zone_state" zone_timeout="{zone_timeout}" zones_file="{zones_file}"
                  spin_after="1" backup_after="2">
      <NavStub/>
      <ClearStub/>
      <SpinStub/>
      <BackUpStub/>
      <AlwaysFailure/>
    </ZoneRecovery>
  </BehaviorTree>
</root>
)";

class ZoneRecoveryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // unique name per test: reusing one name makes rclcpp's rosout logger bookkeeping fail in the
    // teardown of the previous node ("Failed to get logger entry")
    static int serial = 0;
    node_ = std::make_shared<rclcpp::Node>("zone_recovery_test_" + std::to_string(++serial));
    pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/nav_zone_state", rclcpp::QoS(100).reliable().transient_local());
    factory_.registerNodeType<nav_behaviors::ZoneRecovery>("ZoneRecovery");
    factory_.registerSimpleAction("NavStub", failingNav);
    factory_.registerSimpleAction("ClearStub", record("clear"));
    factory_.registerSimpleAction("SpinStub", record("spin"));
    factory_.registerSimpleAction("BackUpStub", record("backup"));
  }

  void TearDown() override
  {
    // g_bb is global and keeps the node alive; leaving it set destroys the node after
    // rclcpp::shutdown() and crashes in the DDS teardown
    g_bb.reset();
    log_.reset();
    pub_.reset();
    node_.reset();
  }

  BT::Tree tree(int nav_fails, const std::string & zones_file = "",
    std::shared_ptr<tf2_ros::Buffer> tf = nullptr)
  {
    log_ = std::make_shared<Log>();
    auto bb = BT::Blackboard::create();
    bb->set<rclcpp::Node::SharedPtr>("node", node_);
    bb->set<LogPtr>("log", log_);
    bb->set("nav_fails", nav_fails);
    bb->set("zone_timeout", 4.0);
    bb->set("zones_file", zones_file);
    bb->set("tf_buffer", tf);
    g_bb = bb;
    return factory_.createTreeFromText(kTree, bb);
  }

  // ticks until the tree stops running; returns the final status
  BT::NodeStatus run(BT::Tree & t, int max_ticks = 100)
  {
    BT::NodeStatus s = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && s == BT::NodeStatus::RUNNING; ++i) {
      s = t.tickOnce();
    }
    return s;
  }

  void publishPhase(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pub_->publish(msg);
    // the BT node drains its own callback group on tick; give DDS a moment to deliver
    for (int i = 0; i < 50 && pub_->get_subscription_count() == 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      rclcpp::spin_some(node_);
    }
    for (int i = 0; i < 5; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      rclcpp::spin_some(node_);
    }
  }

  // Deliver a phase and wait until the tree's ZoneRecovery node has actually seen it, so the test
  // never races the subscription.
  void deliverPhase(BT::Tree & t, const std::string & name)
  {
    auto * rec = zoneRecovery(t);
    ASSERT_NE(rec, nullptr);
    publishPhase(phase(name));
    for (int i = 0; i < 100; ++i) {
      rec->pollZoneState();
      if (rec->zoneFresh(1.0)) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      rclcpp::spin_some(node_);
    }
    ADD_FAILURE() << "zone phase '" << name << "' never reached the BT node";
  }

  static ZoneRecovery * zoneRecovery(BT::Tree & t)
  {
    ZoneRecovery * found = nullptr;
    t.applyVisitor([&found](BT::TreeNode * n) {
        if (!found) {
          found = dynamic_cast<ZoneRecovery *>(n);
        }
    });
    return found;
  }

  std::string phase(const std::string & name, const std::string & zone = "tunnel_1")
  {
    char st[32];
    std::snprintf(st, sizeof(st), " @%.3f", node_->now().seconds());
    return name + " " + zone + st;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  BT::BehaviorTreeFactory factory_;
  LogPtr log_;
};

}  // namespace

TEST_F(ZoneRecoveryTest, LadderEscalatesClearSpinBackUpThenGivesUp)
{
  auto t = tree(20);   // never recovers
  const BT::NodeStatus s = run(t);
  EXPECT_EQ(s, BT::NodeStatus::FAILURE);
  // retries 4 -> attempts 0..3 recover, attempt 4 gives up
  EXPECT_EQ(log_->actions, (std::vector<std::string>{"clear", "spin", "backup", "backup"}));
  EXPECT_EQ(log_->nav_failures, 5);
}

TEST_F(ZoneRecoveryTest, StopsRecoveringOnceNavigationSucceeds)
{
  auto t = tree(2);   // clear, spin, then the third plan works
  EXPECT_EQ(run(t), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(log_->actions, (std::vector<std::string>{"clear", "spin"}));
}

TEST_F(ZoneRecoveryTest, FirstSuccessNeedsNoRecovery)
{
  auto t = tree(0);
  EXPECT_EQ(run(t), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(log_->actions.empty());
  EXPECT_EQ(log_->nav_failures, 0);
}

TEST_F(ZoneRecoveryTest, FinalSuccessClearsRecoveredErrorsWhileFailureRetainsDiagnostics)
{
  auto t = tree(1);
  g_bb->set<std::uint16_t>("smoother_error_code", 503);
  g_bb->set<std::uint16_t>("compute_path_error_code", 208);
  g_bb->set<std::uint16_t>("follow_path_error_code", 103);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(g_bb->get<std::uint16_t>("smoother_error_code"), 503);
  ASSERT_EQ(run(t), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(g_bb->get<std::uint16_t>("smoother_error_code"), 0);
  EXPECT_EQ(g_bb->get<std::uint16_t>("compute_path_error_code"), 0);
  EXPECT_EQ(g_bb->get<std::uint16_t>("follow_path_error_code"), 0);
  auto failed = tree(100);
  g_bb->set<std::uint16_t>("smoother_error_code", 503);
  ASSERT_EQ(run(failed), BT::NodeStatus::FAILURE);
  EXPECT_EQ(g_bb->get<std::uint16_t>("smoother_error_code"), 503);
}

TEST_F(ZoneRecoveryTest, LadderResetsBetweenGoals)
{
  auto t = tree(1);   // one clear, then success
  EXPECT_EQ(run(t), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(log_->actions.size(), 1u);
  // a new goal starts at attempt 0 again, not at spin
  t.haltTree();
  log_->actions.clear();
  g_bb->set("nav_fails", 1);
  EXPECT_EQ(run(t), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(log_->actions, (std::vector<std::string>{"clear"}));
}

TEST_F(ZoneRecoveryTest, NoMotionInsideAZone)
{
  // ALIGN / TRAVERSE / EXIT: the ladder may only clear the costmaps, then it gives up
  for (const char * p : {"align", "traverse", "exit"}) {
    SCOPED_TRACE(p);
    auto t = tree(20);
    deliverPhase(t, p);
    const BT::NodeStatus s = run(t);
    EXPECT_EQ(s, BT::NodeStatus::FAILURE);
    for (const auto & a : log_->actions) {
      EXPECT_EQ(a, "clear") << "recovery moved the robot inside a zone: " << a;
    }
    EXPECT_FALSE(log_->actions.empty());
  }
}

TEST_F(ZoneRecoveryTest, MotionAllowedOnTheApproach)
{
  // APPROACH is outside the opening: backing away from a bad line-up is allowed
  auto t = tree(20);
  deliverPhase(t, "approach");
  EXPECT_EQ(run(t), BT::NodeStatus::FAILURE);
  EXPECT_NE(std::find(log_->actions.begin(), log_->actions.end(), "spin"), log_->actions.end());
  EXPECT_NE(std::find(log_->actions.begin(), log_->actions.end(), "backup"), log_->actions.end());
}

TEST_F(ZoneRecoveryTest, FailureKeepsNoMotionInterlockAfterPhaseExpires)
{
  auto t = tree(20);
  deliverPhase(t, "traverse");
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // select first clear while in zone
  g_bb->set("zone_timeout", 0.0);                   // stale phase on every later tick
  EXPECT_EQ(run(t), BT::NodeStatus::FAILURE);
  EXPECT_EQ(log_->actions, (std::vector<std::string>{"clear", "clear", "clear", "clear"}));
}

TEST_F(ZoneRecoveryTest, NewGoalStartsItsOwnRecoveryLadder)
{
  auto t = tree(20);
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.orientation.w = 1.0;
  g_bb->set("goal", goal);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // clear for first target
  goal.pose.position.x = 2.0;
  g_bb->set("goal", goal);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);  // clear again, not spin
  EXPECT_EQ(log_->actions, (std::vector<std::string>{"clear", "clear"}));
}

TEST_F(ZoneRecoveryTest, PhysicalOccupancyAndMissingTransformPreventMotionWithoutPhase)
{
  const auto path = std::filesystem::temp_directory_path() /
    (std::string(node_->get_name()) + "_zones.yaml");
  {
    std::ofstream file(path);
    file << "zones:\n- {id: t, type: tunnel, cells: 100, centre: [0,0], yaw_deg: 0, size: [1,0.8]}\n";
  }
  auto tf = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  {
    auto t = tree(20, path.string(), tf);  // no transform: cannot prove a recovery is safe
    EXPECT_EQ(run(t), BT::NodeStatus::FAILURE);
    for (const auto & action : log_->actions) {EXPECT_EQ(action, "clear");}
  }
  geometry_msgs::msg::TransformStamped pose;
  pose.header.frame_id = "map";
  pose.child_frame_id = "base_link";
  pose.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(pose, "test", true));
  {
    auto t = tree(20, path.string(), tf);  // physically inside, no phase has ever been sent
    EXPECT_EQ(run(t), BT::NodeStatus::FAILURE);
    for (const auto & action : log_->actions) {EXPECT_EQ(action, "clear");}
  }
  pose.transform.translation.x = 3.0;
  ASSERT_TRUE(tf->setTransform(pose, "test", true));
  {
    auto t = tree(20, path.string(), tf);
    EXPECT_EQ(run(t), BT::NodeStatus::FAILURE);
    EXPECT_NE(std::find(log_->actions.begin(), log_->actions.end(), "spin"), log_->actions.end());
  }
  std::filesystem::remove(path);
}
