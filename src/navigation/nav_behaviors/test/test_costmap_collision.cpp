#include <gtest/gtest.h>
#include "nav_behaviors/costmap_collision.hpp"
#include "nav_behaviors/observed_approach.hpp"
#include "nav_behaviors/overlap_escape.hpp"
#include "nav_behaviors/ordinary_escape_release.hpp"
#include "nav_behaviors/zone_tracker.hpp"

using namespace nav_behaviors;  // NOLINT

namespace
{
std::vector<geometry_msgs::msg::Point> footprint()
{
  std::vector<geometry_msgs::msg::Point> fp;
  nav2_costmap_2d::makeFootprintFromString("[[0.38,0.30],[0.38,-0.30],[-0.30,-0.30],[-0.30,0.30]]", fp);
  return fp;
}
void obstacle(nav2_costmap_2d::Costmap2D & map, double x, double y, unsigned char value = 254)
{
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my));
  map.setCost(mx, my, value);
}
}  // namespace

TEST(CostmapCollision, ChecksInteriorUnknownAndBounds)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.025, -2, -2, 0);
  EXPECT_TRUE(footprintFree(map, footprint(), {0, 0, 0}));
  obstacle(map, 0.0, 0.0);  // entirely inside: a perimeter-only check misses it
  EXPECT_FALSE(footprintFree(map, footprint(), {0, 0, 0}));
  EXPECT_FALSE(footprintFree(map, footprint(), {1.9, 0, 0}));
  map.resetMap(0, 0, 160, 160);
  obstacle(map, 0.1, 0.1, 255);
  EXPECT_FALSE(footprintFree(map, footprint(), {0, 0, 0}));
}

TEST(CostmapCollision, UnknownPassableOnlyWhenAllowed)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.025, -2, -2, nav2_costmap_2d::NO_INFORMATION);
  const auto fp = footprint();
  EXPECT_TRUE(footprintFreeInUnknown(map, fp, {0, 0, 0}));
  EXPECT_FALSE(footprintFree(map, fp, {0, 0, 0}));
  EXPECT_TRUE(footprintMotionFree(map, fp, {0, 0, 0}, {1, 0, 0}, nullptr, true));
  EXPECT_FALSE(footprintMotionFree(map, fp, {0, 0, 0}, {1, 0, 0}));
  obstacle(map, 0.0, 0.0);  // lethal blocks even when unknown is passable
  EXPECT_FALSE(footprintFreeInUnknown(map, fp, {0, 0, 0}));
  EXPECT_FALSE(footprintMotionFree(map, fp, {0, 0, 0}, {1, 0, 0}, nullptr, true));
}

TEST(ObservedApproach, KeepsFootprintInObservedSpaceAndAdvancesAsBoundaryMoves)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.05, -4, -4, nav2_costmap_2d::NO_INFORMATION);
  const Pose2 start{0, 0, 0};
  const Pose2 goal{3.5, 0, 0};
  const auto fp = footprint();
  auto stopped = fp;
  nav2_costmap_2d::padFootprint(stopped, 0.10);
  for (unsigned row = 0; row < 160; ++row) {
    for (unsigned col = 0; col < 120; ++col) {map.setCost(col, row, 0);}
  }
  EXPECT_TRUE(needsObservedApproach(map, fp, goal));
  EXPECT_TRUE(needsObservedApproach(map, fp, {1.9, 0, 0}));
  EXPECT_FALSE(needsObservedApproach(map, fp, {1, 0, 0}));
  const auto first = observedApproach(map, fp, start, goal, 0.10);
  ASSERT_GE(first.size(), 2u);
  EXPECT_GT(first.back().x, 1.45);
  EXPECT_LT(first.back().x, 1.53);
  EXPECT_TRUE(footprintFree(map, stopped, first.back()));
  for (size_t index = 1; index < first.size(); ++index) {
    EXPECT_TRUE(footprintMotionFree(map, fp, first[index - 1], first[index]));
  }
  for (unsigned row = 0; row < 160; ++row) {
    for (unsigned col = 120; col < 150; ++col) {map.setCost(col, row, 0);}
  }
  const auto second = observedApproach(map, fp, first.back(), goal, 0.10);
  ASSERT_GE(second.size(), 2u);
  EXPECT_GT(second.back().x, first.back().x + 1.0);
  EXPECT_GT(goal.x - second.back().x, 0.20);
  for (unsigned row = 0; row < 160; ++row) {
    for (unsigned col = 150; col < 160; ++col) {map.setCost(col, row, 0);}
  }
  EXPECT_FALSE(needsObservedApproach(map, fp, goal));
}

TEST(ObservedApproach, StopsBeforeObstaclesAndRollingWindowEdge)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.05, -4, -4, 0);
  const auto fp = footprint();
  const Pose2 start{0, 0, 0};
  const Pose2 goal{6, 0, 0};
  EXPECT_TRUE(needsObservedApproach(map, fp, goal));
  const auto bounded = observedApproach(map, fp, start, goal, 0.10);
  ASSERT_GE(bounded.size(), 2u);
  EXPECT_GT(bounded.back().x, 3.45);
  EXPECT_LT(bounded.back().x, 3.53);
  obstacle(map, 1.0, 0.0);
  EXPECT_FALSE(needsObservedApproach(map, fp, {1.0, 0.0, 0.0}));
  const auto blocked = observedApproach(map, fp, start, goal, 0.10);
  ASSERT_GE(blocked.size(), 2u);
  EXPECT_LT(blocked.back().x, 0.53);
  obstacle(map, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(observedApproach(map, fp, start, goal, 0.10).empty());
}

TEST(ObservedApproach, KnownGoalBeyondUnknownGap)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.05, -4, -4, 0);
  const auto fp = footprint();
  const Pose2 start{0, 0, 0};
  const Pose2 goal{3.5, 0, 0};
  EXPECT_TRUE(observedApproach(map, fp, start, goal, 0.10, true).empty());
  obstacle(map, 1.5, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(observedApproach(map, fp, start, goal, 0.10, true).empty());
  obstacle(map, 1.5, 0, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(needsObservedApproach(map, fp, goal));
  const auto approach = observedApproach(map, fp, start, goal, 0.10, true);
  ASSERT_GE(approach.size(), 2u);
  EXPECT_GT(approach.back().x, .95);
  EXPECT_LT(approach.back().x, 1.03);
}

TEST(CostmapCollision, ReentryCoversRotatingFootprintReactionAndBrakingInEveryDirection)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .025, -2.5, -2.5, 0);
  const Pose2 robot{0, 0, 1.3};
  EXPECT_TRUE(clearBrakingEnvelope(map, footprint(), robot, .8, .9, .6));
  for (unsigned char cost : {253, 254, 255}) {
    obstacle(map, -.8, .6, cost);
    EXPECT_FALSE(clearBrakingEnvelope(map, footprint(), robot, .8, .9, .6));
    map.resetMap(0, 0, 200, 200);
  }
  EXPECT_FALSE(clearBrakingEnvelope(map, footprint(), {2., 0., 0.}, .8, .9, .6));
  EXPECT_FALSE(clearBrakingEnvelope(map, footprint(), robot, .8, 0., .6));
  EXPECT_FALSE(clearBrakingEnvelope(map, {}, robot, .8, .9, .6));
}

TEST(CostmapCollision, ChecksBetweenSparsePosesAndDuringRotation)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.025, -2, -2, 0);
  obstacle(map, 0, 0);
  EXPECT_TRUE(footprintFree(map, footprint(), {-1, 0, 0}));
  EXPECT_TRUE(footprintFree(map, footprint(), {1, 0, 0}));
  EXPECT_FALSE(footprintMotionFree(map, footprint(), {-1, 0, 0}, {1, 0, 0}));
  map.resetMap(0, 0, 160, 160);
  obstacle(map, 0.43, 0);
  EXPECT_TRUE(footprintFree(map, footprint(), {0, 0, 0}));
  EXPECT_TRUE(footprintFree(map, footprint(), {0, 0, 1.57079632679}));
  EXPECT_FALSE(footprintMotionFree(map, footprint(), {0, 0, 0}, {0, 0, 1.57079632679}));
}

TEST(CostmapCollision, UsesActualHeadingForAsymmetricFootprint)
{
  nav2_costmap_2d::Costmap2D map(160, 160, 0.025, -2, -2, 0);
  obstacle(map, -0.36, 0.0);
  EXPECT_TRUE(footprintFree(map, footprint(), {0, 0, 0}));
  EXPECT_FALSE(footprintFree(map, footprint(), {0, 0, 3.14159265359}));
}

TEST(CostmapCollision, TrackerKeepsCollisionValidatedHeadingWhenApproachFacesBackwards)
{
  nav2_costmap_2d::Costmap2D map(240, 160, .025, -3., -2., 0);
  // A corner behind the path is clear of the short rear, but not the long nose
  // after a pi flip. The shaper validated yaw zero, regardless of arrival yaw.
  obstacle(map, -.36, .29);
  Zone zone; zone.depth = .35; zone.width = .8;
  ZoneTracker tracker; tracker.setZones({zone});
  std::vector<Pose2> path;
  for (int i = 0; i <= 200; ++i) {path.push_back({-2.5 + .025 * i, 0., 0.});}
  tracker.setPath(path);
  const auto entry = tracker.compute({-1.9, 0., 2.5});
  EXPECT_NEAR(entry.lock, 0., 1e-9);
  tracker.compute({0., 0., 0.});
  const auto ahead = tracker.ahead(.1);
  ASSERT_FALSE(ahead.empty());
  for (const auto & q : ahead) {
    EXPECT_NEAR(q.yaw, 0., 1e-9);
    EXPECT_TRUE(footprintFree(map, footprint(), q));
  }
}

TEST(CostmapCollision, ShaperFindsNarrowFreeBandWithPlanningReserve)
{
  nav2_costmap_2d::Costmap2D map(240, 160, .025, -3., -2., 0);
  for (unsigned y = 0; y < map.getSizeInCellsY(); ++y) {
    for (unsigned x = 0; x < map.getSizeInCellsX(); ++x) {
      double wx, wy; map.mapToWorld(x, y, wx, wy);
      if (wy > .35 || wy < -.35 || (wx > .4 && wx < 1. && wy < -.30)) {
        map.setCost(x, y, 254);
      }
    }
  }
  auto padded = footprint(); nav2_costmap_2d::padFootprint(padded, .0225);
  Zone zone; zone.id = "tunnel"; zone.depth = .35; zone.width = .8;
  std::vector<Pose2> input;
  for (int i = 0; i <= 200; ++i) {input.push_back({-2.5 + i * .025, 0., 0.});}
  ShaperParams params; params.max_body_offset = .06; params.offset_step = .005;
  params.axis_margin = 1.5;  // retain the offset until the complete chassis clears the wall spur
  std::vector<ShapeEvent> events;
  auto output = shapePath(input, {zone}, params,
    [&](const Pose2 & q) {return footprintFree(map, padded, q);}, &events);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_TRUE(events[0].shaped) << events[0].reason;
  EXPECT_NEAR(events[0].v_body, .025, .0001);
  for (size_t i = 1; i < output.size(); ++i) {
    EXPECT_TRUE(footprintMotionFree(map, padded, output[i - 1], output[i]));
  }
}

TEST(CostmapCollision, LateralAlignmentCanClearACornerBeforeForwardAdvance)
{
  nav2_costmap_2d::Costmap2D map(240, 160, .025, -3., -2., 0);
  for (unsigned y = 0; y < map.getSizeInCellsY(); ++y) {
    for (unsigned x = 0; x < map.getSizeInCellsX(); ++x) {
      double wx, wy; map.mapToWorld(x, y, wx, wy);
      if (wy > .4 || wy < -.4 || (wx > .7 && wx < 1.3 && wy < -.30)) {
        map.setCost(x, y, 254);
      }
    }
  }
  auto fp = footprint(); nav2_costmap_2d::padFootprint(fp, .01);
  Zone zone; zone.depth = .35; zone.width = .8;
  ZoneTracker tracker; tracker.setZones({zone});
  std::vector<Pose2> path;
  for (int i = 0; i <= 160; ++i) {path.push_back({-2. + .025 * i, .025, M_PI});}
  tracker.setPath(path);
  Pose2 robot{.35, -.04, M_PI};
  auto integrate = [](const Pose2 & r, double vx, double vy, double dt) {
      return Pose2{r.x + dt * (std::cos(r.yaw) * vx - std::sin(r.yaw) * vy),
        r.y + dt * (std::sin(r.yaw) * vx + std::cos(r.yaw) * vy), r.yaw};
    };
  ASSERT_TRUE(footprintFree(map, fp, robot));
  auto output = tracker.compute(robot);
  EXPECT_FALSE(footprintMotionFree(map, fp, robot, integrate(robot, output.vx, output.vy, .25)));
  for (int i = 0; i < 15; ++i) {
    output = tracker.compute(robot);
    const auto aligned = integrate(robot, output.align_vx, output.align_vy, .1);
    ASSERT_TRUE(footprintMotionFree(map, fp, robot, aligned));
    EXPECT_NEAR(aligned.x, robot.x, 1e-9);
    robot = aligned;
  }
  EXPECT_NEAR(robot.y, .025, .001);
  output = tracker.compute(robot);
  EXPECT_TRUE(footprintMotionFree(map, fp, robot, integrate(robot, output.vx, output.vy, .25)));
}


TEST(OverlapEscape, OccupiedCenterFindsFreeExitWithoutClearingTheMap)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .025, -2, -2, 0);
  obstacle(map, 0, 0);
  const auto paths = escapePaths(map, footprint(), {0, 0, 0}, {1.5, 0, 0});
  ASSERT_FALSE(paths.empty());
  EXPECT_FALSE(footprintFree(map, footprint(), {0, 0, 0}));
  for (const auto & path : paths) {
    EXPECT_TRUE(footprintFree(map, footprint(), path.poses.back()));
    EXPECT_TRUE(escapeMotionFree(map, footprint(), path.poses.front(), path.poses.back()));
    EXPECT_NEAR(path.poses.back().yaw, 0, 1e-12);
  }
  unsigned x, y; map.worldToMap(0, 0, x, y);
  EXPECT_EQ(map.getCost(x, y), 254);
}

TEST(OverlapEscape, RejectsNewContactsRotationUnknownAndGrowingOverlap)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .025, -2, -2, 0);
  obstacle(map, -.29, 0);  // initial contact at the rear
  EXPECT_TRUE(escapeMotionFree(map, footprint(), {0, 0, 0}, {.6, 0, 0}));
  EXPECT_FALSE(escapeMotionFree(map, footprint(), {.0125, 0, 0}, {-.0075, 0, 0}));
  EXPECT_FALSE(escapeMotionFree(map, footprint(), {0, 0, 0}, {.6, 0, .1}));
  obstacle(map, .6, 0);  // must not exempt a second obstacle ahead
  EXPECT_FALSE(escapeMotionFree(map, footprint(), {0, 0, 0}, {.6, 0, 0}));
  map.resetMap(0, 0, 160, 160); obstacle(map, 0, 0, 255);
  EXPECT_TRUE(escapePaths(map, footprint(), {0, 0, 0}, {1, 0, 0}).empty());
  EXPECT_TRUE(escapePaths(map, footprint(), {1.9, 0, 0}, {1, 0, 0}).empty());
}

TEST(OrdinaryEscapeRelease, RequiresDistinctFreshMapAndObservationStamps)
{
  ClearMapConfirmation confirm;
  constexpr int64_t start = 1000000000, step = 100000000;
  EXPECT_FALSE(confirm.update(true, start, start, start));
  EXPECT_FALSE(confirm.update(true, start + 2*step, start, start + 2*step));
  EXPECT_FALSE(confirm.update(true, start + 3*step, start, start + 3*step));
  confirm.reset();
  EXPECT_FALSE(confirm.update(true, start, start, start));
  EXPECT_FALSE(confirm.update(true, start + 2*step, start + 2*step, start));
  for (int index = 3; index < 10; ++index) {
    const auto stamp = start + index*step;
    EXPECT_FALSE(confirm.update(true, stamp, stamp, stamp));
  }
  EXPECT_TRUE(confirm.update(true, start + 10*step, start + 10*step, start + 10*step));
  EXPECT_FALSE(confirm.update(true, start + 16*step, start + 16*step, start + 10*step));
  EXPECT_FALSE(confirm.update(true, start + 17*step, start + 17*step, start + 17*step));
}

TEST(OrdinaryEscapeRelease, OccupancyClockGapsAndRewindsInvalidateClearance)
{
  ClearMapConfirmation confirm;
  constexpr int64_t start = 1000000000, step = 100000000;
  auto sample = [&](bool clear, int64_t stamp) {return confirm.update(clear, stamp, stamp, stamp);};
  EXPECT_FALSE(sample(true, start));
  EXPECT_FALSE(sample(false, start + 2*step));
  EXPECT_FALSE(sample(true, start + 3*step));
  for (int index = 4; index < 13; ++index) {EXPECT_FALSE(sample(true, start + index*step));}
  EXPECT_TRUE(sample(true, start + 13*step));
  EXPECT_FALSE(sample(true, start + 20*step));
  EXPECT_FALSE(sample(true, start));
  EXPECT_FALSE(confirm.update(true, start + step, start + 2*step, start + step));
  EXPECT_FALSE(confirm.update(true, start + 2*step, 0, start + 2*step));
}

TEST(OrdinaryEscapeRelease, RechecksConnectorAndSweptPathWithoutSkippingCorners)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .025, -2., -2., 0);
  const Pose2 robot{0., -.8, .3};
  const std::vector<XY> path{{-1., 0.}, {0., 0.}, {.85, 0.}, {.85, 1.}};
  const auto fp = footprint();
  EXPECT_TRUE(escapeRejoinPathFree(map, fp, path, robot, 1.4));
  obstacle(map, 0., -.2);
  EXPECT_FALSE(escapeRejoinPathFree(map, fp, path, robot, 1.4));
  map.resetMap(0, 0, 200, 200);
  obstacle(map, .85, .4, 255);
  EXPECT_FALSE(escapeRejoinPathFree(map, fp, path, robot, 1.4));
  map.resetMap(0, 0, 200, 200);
  obstacle(map, .85, .0, 253);
  EXPECT_FALSE(escapeRejoinPathFree(map, fp, path, robot, 1.4));
  EXPECT_FALSE(escapeRejoinPathFree(map, fp, {}, robot, .3));
  EXPECT_FALSE(escapeRejoinPathFree(map, fp, {{2.9, -.8}}, robot, .3));
}

TEST(OrdinaryEscapeHold, OccupiedFootprintEscapesAtOneSecondWithoutRearming)
{
  using Decision = OrdinaryEscapeHold::Decision;
  using namespace std::chrono_literals;
  OrdinaryEscapeHold hold;
  const auto start = OrdinaryEscapeHold::Clock::time_point{};
  constexpr int64_t stamp = 1000000000;
  hold.start(start);
  EXPECT_EQ(hold.update(start + 999ms, true, false, stamp, stamp, stamp), Decision::Wait);
  EXPECT_EQ(hold.update(start + 1s, true, false, stamp, stamp, stamp), Decision::Escape);
  EXPECT_TRUE(hold.active());
  EXPECT_EQ(hold.update(start + 2s, true, false, stamp, stamp, stamp), Decision::Escape);
  hold.reset();
  EXPECT_FALSE(hold.active());
  hold.start(start + 2s);
  EXPECT_EQ(hold.update(start + 2500ms, true, false, stamp, stamp, stamp), Decision::Wait);
}

TEST(OrdinaryEscapeHold, TimeoutDoesNotAuthorizeEscapeOnUnusableData)
{
  using Decision = OrdinaryEscapeHold::Decision;
  using namespace std::chrono_literals;
  OrdinaryEscapeHold hold;
  const auto start = OrdinaryEscapeHold::Clock::time_point{};
  constexpr int64_t stamp = 1000000000;
  hold.start(start);
  EXPECT_EQ(hold.update(start + 1s, false, false, stamp, stamp, stamp), Decision::Fail);
  EXPECT_EQ(hold.update(start + 1s, true, false, stamp, 0, stamp), Decision::Fail);
  EXPECT_EQ(hold.update(start + 1s, true, false, stamp, stamp, stamp - 500000001), Decision::Fail);
}

TEST(OrdinaryEscapeHold, ClearFootprintStillNeedsFreshConfirmationWithABoundedGrace)
{
  using Decision = OrdinaryEscapeHold::Decision;
  using namespace std::chrono_literals;
  OrdinaryEscapeHold hold;
  const auto start = OrdinaryEscapeHold::Clock::time_point{};
  constexpr int64_t stamp = 1000000000, step = 100000000;
  hold.start(start);
  for (int index = 2; index < 12; ++index) {
    const auto sample = stamp + index * step;
    EXPECT_EQ(hold.update(start + index * 100ms, true, true, sample, sample, sample), Decision::Wait);
  }
  constexpr auto confirmed = stamp + 12 * step;
  EXPECT_EQ(hold.update(start + 1200ms, true, true, confirmed, confirmed, confirmed), Decision::Rejoin);

  hold.reset();
  hold.start(start);
  EXPECT_EQ(hold.update(start + 2s, true, true, stamp, stamp, stamp), Decision::Wait);
  EXPECT_EQ(hold.update(start + 2250ms, true, true, stamp, stamp, stamp), Decision::Fail);
}

TEST(OrdinaryEscapeHold, EscapeCommandAllowsClearanceConfirmationAfterTranslation)
{
  using Decision = OrdinaryEscapeHold::Decision;
  using namespace std::chrono_literals;
  OrdinaryEscapeHold hold;
  const auto start = OrdinaryEscapeHold::Clock::time_point{};
  constexpr int64_t stamp = 1000000000, step = 100000000;
  hold.start(start);
  EXPECT_EQ(hold.update(start + 3s, true, false, stamp, stamp, stamp), Decision::Escape);
  hold.commandedEscape(start + 3s);
  for (int index = 1; index < 11; ++index) {
    const auto sample = stamp + index * step;
    EXPECT_EQ(hold.update(start + 3s + index * 100ms, true, true, sample, sample, sample), Decision::Wait);
  }
  constexpr auto confirmed = stamp + 11 * step;
  EXPECT_EQ(hold.update(start + 4100ms, true, true, confirmed, confirmed, confirmed), Decision::Rejoin);
}

TEST(OverlapEscape, LeavingContactCannotReenterAndRollingOriginDoesNotChangeIdentity)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .025, -2, -2, 0);
  obstacle(map, -.29, 0);
  const auto initial = footprintContacts(map, footprint(), {0, 0, 0});
  map.updateOrigin(-1.9, -2.0);
  const auto shifted = footprintContacts(map, footprint(), {0, 0, 0});
  EXPECT_EQ(initial.cells, shifted.cells);
  EXPECT_TRUE(contactsDecrease(initial, shifted));
  const auto free = footprintContacts(map, footprint(), {.6, 0, 0});
  EXPECT_TRUE(contactsDecrease(initial, free));
  EXPECT_FALSE(contactsDecrease(free, initial));
}


TEST(OverlapEscape, EmbeddedStartLeavesOriginalThickObstacle)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .025, -2, -2, 0);
  for (double x = -.5; x <= .5; x += .02) {
    for (double y = -1.; y <= 1.; y += .02) {obstacle(map, x, y);}
  }
  const auto paths = escapePaths(map, footprint(), {0, 0, 0}, {1.5, 0, 0});
  ASSERT_FALSE(paths.empty());
  EXPECT_GT(paths.front().poses.back().x, .8);
  EXPECT_TRUE(footprintFree(map, footprint(), paths.front().poses.back()));
  EXPECT_TRUE(escapeMotionFree(map, footprint(), paths.front().poses.front(), paths.front().poses.back()));
}


TEST(OverlapEscape, InflationOnlyStartStillFindsACollisionFreePrefix)
{
  nav2_costmap_2d::Costmap2D map(160, 160, .025, -2, -2, 0);
  obstacle(map, 0, 0, 253);
  ASSERT_TRUE(footprintFree(map, footprint(), {0, 0, 0}));
  const auto exits = escapePaths(map, footprint(), {0, 0, 0}, {1, 0, 0});
  ASSERT_FALSE(exits.empty());
  EXPECT_TRUE(footprintMotionFree(map, footprint(), exits.front().poses.front(), exits.front().poses.back()));
}
