#include <gtest/gtest.h>
#include "nav_behaviors/translation_goal_checker.hpp"

TEST(TranslationGoalChecker, RequiresStoppedXYWithoutRequiringYaw)
{
  nav_behaviors::TranslationGoalChecker checker;
  geometry_msgs::msg::Pose query, goal;
  geometry_msgs::msg::Twist speed;
  query.position.x = .1;
  query.orientation.z = 1.; goal.orientation.w = 1.;
  speed.angular.z = 2.;  // Ordinary mode cannot command or stop yaw.
  EXPECT_TRUE(checker.isGoalReached(query, goal, speed));
  speed.linear.x = .2;
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  speed.linear.x = 0.; speed.linear.y = -.2;
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  speed.linear.x = speed.linear.y = .04;  // Vector speed exceeds .05.
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  speed.linear.x = speed.linear.y = .03;
  EXPECT_TRUE(checker.isGoalReached(query, goal, speed));
}

TEST(TranslationGoalChecker, DoesNotLatchAnEarlierPositionEntry)
{
  nav_behaviors::TranslationGoalChecker checker;
  geometry_msgs::msg::Pose query, goal;
  geometry_msgs::msg::Twist speed;
  query.position.x = .14; speed.linear.x = .2;
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  query.position.x = .2; speed.linear.x = 0.;
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  query.position.x = .1;
  EXPECT_TRUE(checker.isGoalReached(query, goal, speed));
  query.position.x = .2;
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  checker.reset();
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
}

TEST(TranslationGoalChecker, RejectsNonFinitePositionOrTranslationSpeed)
{
  nav_behaviors::TranslationGoalChecker checker;
  geometry_msgs::msg::Pose query, goal;
  geometry_msgs::msg::Twist speed;
  speed.linear.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
  speed.linear.y = 0.; query.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(checker.isGoalReached(query, goal, speed));
}

TEST(TranslationGoalChecker, ReportsOnlyXYPositionAndTranslationTolerances)
{
  nav_behaviors::TranslationGoalChecker checker;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist speed;
  ASSERT_TRUE(checker.getTolerances(pose, speed));
  EXPECT_DOUBLE_EQ(pose.position.x, .15);
  EXPECT_DOUBLE_EQ(pose.position.y, .15);
  EXPECT_DOUBLE_EQ(speed.linear.x, .05);
  EXPECT_DOUBLE_EQ(speed.linear.y, .05);
  EXPECT_DOUBLE_EQ(pose.orientation.w, std::numeric_limits<double>::lowest());
  EXPECT_DOUBLE_EQ(speed.angular.z, std::numeric_limits<double>::lowest());
}
