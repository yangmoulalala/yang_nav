#include <gtest/gtest.h>
#include "nav_behaviors/costmap_collision.hpp"
#include "nav_behaviors/overlap_escape.hpp"
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
