#include <gtest/gtest.h>
#include "nav_behaviors/translation_reference.hpp"
using nav_behaviors::translationReference;
using nav_behaviors::XY;
TEST(TranslationReference, CorrectsCrossTrackAndUsesArcLengthAtCorner)
{
  const auto straight = translationReference({{0,0},{2,0}}, {.5,.1}, .25, 1., .25);
  EXPECT_GT(straight.x, 0.2); EXPECT_LT(straight.y, -0.08);
  const auto corner = translationReference({{0,0},{1,0},{1,1}}, {.9,0}, .3, 1., 1.);
  EXPECT_NEAR(corner.x,.1,1e-9); EXPECT_NEAR(corner.y,.2,1e-9);
}
TEST(TranslationReference, StopsAtGoalAndHandlesShortRepeatedOrEmptyPaths)
{
  const auto stop = translationReference({{0,0},{1,0}}, {1,0}, .25, 1., .25);
  EXPECT_EQ(stop.x,0.); EXPECT_EQ(stop.y,0.);
  const auto end = translationReference({{1,0},{1,0}}, {.9,0}, .25, 1., .25);
  EXPECT_NEAR(end.x,.1,1e-9); EXPECT_EQ(end.y,0.);
  const auto empty = translationReference({}, {1,2}, .25, 1., .25);
  EXPECT_EQ(empty.x,0.); EXPECT_EQ(empty.y,0.);
}

TEST(TranslationReference, ShortDirectionPreviewDoesNotLowerStraightCruiseSpeed)
{
  const auto straight = translationReference({{0, 0}, {2, 0}}, {0, 0}, 2., 1., .7, .55);
  EXPECT_NEAR(straight.x, .7, 1e-9);
  EXPECT_NEAR(straight.y, 0., 1e-9);
  const auto corner = translationReference({{0, 0}, {1, 0}, {1, 1}}, {0, 0}, 2., 1., .7, .55);
  EXPECT_NEAR(corner.x, .7, 1e-9);
  EXPECT_NEAR(corner.y, 0., 1e-9);
  const auto approach = translationReference({{0, 0}, {2, 0}}, {1.95, 0}, 2., 1., .7, .55);
  EXPECT_NEAR(approach.x, .05, 1e-9);
}

TEST(TranslationReference, RejectsContinuedDepartureButAllowsTrackingAndReentry)
{
  const std::vector<XY> path{{-2.41, -1.97}, {-1.42, -.99}, {-.65, -.6}};
  EXPECT_TRUE(nav_behaviors::movesAwayFromPath(path, {-2.19, -1.97}, {.67, .08}, .15));
  EXPECT_TRUE(nav_behaviors::movesAwayFromPath(path, {-1.62, -2.16}, {.72, -.24}, .15));
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {-1.62, -2.16}, {.1, .6}, .15));
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {-2.41, -1.97}, {.5, .5}, .15));
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {-1.62, -2.16}, {}, .15));
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {-2.19, -1.97}, {.67, .08}, 0.));
}

TEST(TranslationReference, RejectsContinuedDriftBeyondTheLimit)
{
  const std::vector<XY> path{{0, 0}, {2, 0}};
  EXPECT_TRUE(nav_behaviors::movesAwayFromPath(path, {1., .1}, {.25, .20}, .15));
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {1., .1}, {.25, 0.}, .15));
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {1., .1}, {.25, -.20}, .15));
}

TEST(TranslationReference, ToleratesSmallOutwardComponentWhenAlreadyOffPath)
{
  // A flat 5 mm deadband over the 0.35 s preview is a 14 mm/s outward-speed limit:
  // every lateral component tripped it and the controller threw instead of steering
  // back. The deadband now scales with the offset, so small drift converges.
  const std::vector<XY> path{{0, 0}, {2, 0}};
  EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, {1., .20}, {.25, .03}, .15));
  EXPECT_TRUE(nav_behaviors::movesAwayFromPath(path, {1., .20}, {.25, .30}, .15));
}

TEST(TranslationReference, GuardedReferenceRejoinsWithoutOvershootingTheOppositeSide)
{
  const std::vector<XY> path{{0., 0.}, {3., 0.}};
  for (double error : {-.09, -.06, .06, .09}) {
    const XY robot{.5, error};
    const auto original = translationReference(path, robot, 2., 1., 1., .10);
    EXPECT_TRUE(nav_behaviors::movesAwayFromPath(path, robot, original, .07));
    const auto revised = translationReference(path, robot, 2., 1., 1., .10, nav_behaviors::pathDeparturePreview);
    EXPECT_FALSE(nav_behaviors::movesAwayFromPath(path, robot, revised, .07));
    EXPECT_GT(revised.x, .9);
    EXPECT_LT(error * revised.y, 0.);
    EXPECT_LE(std::hypot(revised.x, revised.y), 1. + 1e-12);
  }
}

TEST(TranslationReference, GuardedReferenceDoesNotExtendBeyondCornerOrAccelerateAtGoal)
{
  const std::vector<XY> path{{0., 0.}, {1., 0.}, {1., 1.}};
  const XY robot{.9, 0.};
  const auto corner = translationReference(path, robot, 2., 1., 1., .10, nav_behaviors::pathDeparturePreview);
  EXPECT_NEAR(robot.x + nav_behaviors::pathDeparturePreview * corner.x, 1., 1e-9);
  EXPECT_NEAR(robot.y + nav_behaviors::pathDeparturePreview * corner.y, .25, 1e-9);
  const auto end = translationReference(path, {1., .95}, 2., 1., 1., .10, nav_behaviors::pathDeparturePreview);
  EXPECT_NEAR(end.y, .05, 1e-9);
}

TEST(TranslationReference, CurvatureLimitsBendSpeedWithoutPenalizingStraightTravel)
{
  const std::vector<XY> path{{0., 0.}, {1., 0.}, {1., 2.}};
  const auto corner = translationReference(path, {.825, 0.}, 2., 1., 1., .10, .35, .9);
  EXPECT_GT(std::hypot(corner.x, corner.y), .2);
  EXPECT_LT(std::hypot(corner.x, corner.y), .4);
  const auto straight = translationReference(path, {.15, 0.}, 2., 1., 1., .10, .35, .9);
  EXPECT_DOUBLE_EQ(straight.x, 1.);
}

TEST(TranslationReference, RotatingAndTranslatingTheMapRotatesTheReference)
{
  const double yaw = 1.3, c = std::cos(yaw), s = std::sin(yaw);
  auto transform = [&](XY a) {return XY{3+c*a.x-s*a.y,-2+s*a.x+c*a.y};};
  const auto original = translationReference({{0,0},{2,0}}, {.5,.1}, .25, 1., .25);
  const auto rotated = translationReference({transform({0,0}),transform({2,0})}, transform({.5,.1}), .25, 1., .25);
  EXPECT_NEAR(rotated.x,c*original.x-s*original.y,1e-9);
  EXPECT_NEAR(rotated.y,s*original.x+c*original.y,1e-9);
}
TEST(TranslationReference, FeedbackToleratesBothSignsOfVelocityHeadingError)
{
  for (double yaw : {-0.35,0.35}) {
    XY robot{0,.05}; const std::vector<XY> path{{0,0},{2,0}};
    double max_cross = 0.;
    for (int i=0;i<1500;++i) {
      auto v = translationReference(path,robot,.25,1.,.25);
      robot.x += .02*1.1*(std::cos(yaw)*v.x-std::sin(yaw)*v.y);
      robot.y += .02*1.1*(std::sin(yaw)*v.x+std::cos(yaw)*v.y);
      max_cross = std::max(max_cross,std::abs(robot.y));
    }
    EXPECT_LT(max_cross,.1); EXPECT_LT(std::hypot(robot.x-2,robot.y),.01);
  }
}
