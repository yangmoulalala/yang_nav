#include <gtest/gtest.h>

#include <string>
#include <limits>

#include "nav_behaviors/zone_state.hpp"

using namespace nav_behaviors;  // NOLINT

TEST(ZoneState, PhasesOnTheApproach)
{
  const double align = 0.8;
  EXPECT_EQ(zonePhaseOf(-2.0, 0.35, align), ZonePhase::Approach);   // window start
  EXPECT_EQ(zonePhaseOf(-0.81, 0.35, align), ZonePhase::Approach);  // still free to replan
  EXPECT_EQ(zonePhaseOf(-0.79, 0.35, align), ZonePhase::Align);     // heading locked from here
  EXPECT_EQ(zonePhaseOf(-0.001, 0.35, align), ZonePhase::Align);
  EXPECT_EQ(zonePhaseOf(0.0, 0.35, align), ZonePhase::Traverse);    // front of the body
  EXPECT_EQ(zonePhaseOf(0.349, 0.35, align), ZonePhase::Traverse);
  EXPECT_EQ(zonePhaseOf(0.351, 0.35, align), ZonePhase::Exit);
  EXPECT_EQ(zonePhaseOf(1.0, 0.35, align), ZonePhase::Exit);
}

TEST(ZoneState, ReplanPausedOnlyWhenTrackerOwnsPath)
{
  EXPECT_FALSE(phasePausesReplan(ZonePhase::None));
  EXPECT_FALSE(phasePausesReplan(ZonePhase::Approach));
  EXPECT_TRUE(phasePausesReplan(ZonePhase::Align));
  EXPECT_TRUE(phasePausesReplan(ZonePhase::Traverse));
  EXPECT_TRUE(phasePausesReplan(ZonePhase::Exit));
}

TEST(ZoneState, ParsePublishedPhase)
{
  EXPECT_EQ(parseZonePhase("none"), ZonePhase::None);
  EXPECT_EQ(parseZonePhase(""), ZonePhase::None);
  EXPECT_EQ(parseZonePhase("approach tunnel_3"), ZonePhase::Approach);
  EXPECT_EQ(parseZonePhase("align tunnel_3"), ZonePhase::Align);
  EXPECT_EQ(parseZonePhase("traverse tunnel_3"), ZonePhase::Traverse);
  EXPECT_EQ(parseZonePhase("exit tunnel_3"), ZonePhase::Exit);
  EXPECT_EQ(parseZonePhase("garbage"), ZonePhase::None);
}

TEST(ZoneState, PhaseNamesRoundTrip)
{
  for (auto p : {ZonePhase::None, ZonePhase::Approach, ZonePhase::Align, ZonePhase::Traverse, ZonePhase::Exit}) {
    EXPECT_EQ(parseZonePhase(zonePhaseName(p)), p);
  }
  EXPECT_STREQ(zonePhaseName(ZonePhase::Traverse), "traverse");
}

TEST(ZoneState, RecoveryBlockedFromAlign)
{
  // Phase policy: no spin/back-up from ALIGN; physical occupancy is checked separately in ZoneRecovery.
  // The approach phase alone does not inhibit motion recovery.
  EXPECT_FALSE(phaseBlocksRecovery(ZonePhase::None));
  EXPECT_FALSE(phaseBlocksRecovery(ZonePhase::Approach));
  EXPECT_TRUE(phaseBlocksRecovery(ZonePhase::Align));
  EXPECT_TRUE(phaseBlocksRecovery(ZonePhase::Traverse));
  EXPECT_TRUE(phaseBlocksRecovery(ZonePhase::Exit));
}

TEST(ZoneState, PhaseMemoryTracksTheNewestSample)
{
  PhaseMemory m;
  EXPECT_FALSE(m.fresh(100.0, 1.5));
  m.update(ZonePhase::Approach, 10.0);
  EXPECT_TRUE(m.fresh(11.0, 1.5));
  EXPECT_FALSE(m.pausedNow(11.0, 1.5));
  EXPECT_FALSE(m.blockedRecently(11.0, 4.0));

  m.update(ZonePhase::Traverse, 11.0);
  EXPECT_TRUE(m.pausedNow(11.2, 1.5));   // traverse pauses replanning too
  EXPECT_TRUE(m.blockedRecently(11.2, 4.0));

  // an out-of-order replay must not move the state backwards
  m.update(ZonePhase::None, 10.5);
  EXPECT_TRUE(m.blockedRecently(11.2, 4.0));
  EXPECT_TRUE(m.pausedNow(11.2, 1.5));

  // the controller resets to none when it fails: freshness expires, the last pausing phase stays
  m.update(ZonePhase::None, 12.0);
  EXPECT_TRUE(m.fresh(12.5, 1.5));
  EXPECT_FALSE(m.pausedNow(12.5, 1.5));
  EXPECT_TRUE(m.blockedRecently(12.5, 1.5));
  EXPECT_FALSE(m.blockedRecently(14.0, 1.5));
}

TEST(ZoneState, PhaseMemoryIgnoresUnstampedSamples)
{
  PhaseMemory m;
  m.update(ZonePhase::Traverse, 0.0);   // old format: freshness unknown
  EXPECT_FALSE(m.fresh(1.0, 1.5));
  EXPECT_FALSE(m.blockedRecently(1.0, 4.0));
}

TEST(ZoneState, InvalidAndFutureTimestampsCannotPauseNavigation)
{
  PhaseMemory m;
  m.update(ZonePhase::Traverse, std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(m.have_phase);
  m.update(ZonePhase::Traverse, std::numeric_limits<double>::infinity());
  EXPECT_FALSE(m.have_phase);
  m.update(ZonePhase::Traverse, 100.0);
  EXPECT_FALSE(m.fresh(90.0, 1.5));
  EXPECT_FALSE(m.pausedNow(90.0, 1.5));
  EXPECT_FALSE(m.blockedRecently(90.0, 4.0));
}

TEST(ZoneState, BlockedPathReplansWithoutEnablingMotionRecovery)
{
  EXPECT_EQ(parseZonePhase("blocked tunnel_3 @12"), ZonePhase::Blocked);
  EXPECT_FALSE(phasePausesReplan(ZonePhase::Blocked));
  EXPECT_TRUE(phaseBlocksRecovery(ZonePhase::Blocked));
  PhaseMemory memory;
  memory.update(ZonePhase::Blocked, 12.0);
  EXPECT_TRUE(memory.path_blocked);
  EXPECT_FALSE(memory.pausedNow(12.1, 1.5));
  EXPECT_TRUE(memory.blockedRecently(12.1, 4.0));
  memory.update(ZonePhase::Traverse, 12.2);
  EXPECT_FALSE(memory.path_blocked);
  EXPECT_TRUE(memory.pausedNow(12.3, 1.5));
}
