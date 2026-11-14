#include "nav_behaviors/zone_state.hpp"

#include <stdexcept>
#include <cmath>

namespace nav_behaviors
{

ZonePhase zonePhaseOf(double s_rel, double body, double align_before)
{
  // [body, +∞) -> Exit
  if (s_rel >= body) {
    return ZonePhase::Exit;
  }
  // [0, body) -> Traverse
  if (s_rel >= 0.0) {
    return ZonePhase::Traverse;
  }
  // (-align_before, 0) -> Align
  if (s_rel > -align_before) {
    return ZonePhase::Align;
  }
  // (-∞, -align_before] -> Approach
  return ZonePhase::Approach;
}

ZonePhase parseZonePhase(const std::string & text, double * stamp)
{
  const std::size_t sp = text.find(' ');
  const std::string head = text.substr(0, sp == std::string::npos ? text.size() : sp);
  if (stamp) {
    *stamp = 0.0;
    const std::size_t at = text.rfind(" @");
    if (at != std::string::npos) {
      try {
        *stamp = std::stod(text.substr(at + 2));
      } catch (const std::exception &) {
        *stamp = 0.0;
      }
    }
  }
  if (head == "approach") {return ZonePhase::Approach;}
  if (head == "align") {return ZonePhase::Align;}
  if (head == "traverse") {return ZonePhase::Traverse;}
  if (head == "exit") {return ZonePhase::Exit;}
  if (head == "blocked") {return ZonePhase::Blocked;}
  return ZonePhase::None;
}

void PhaseMemory::update(ZonePhase p, double stamp)
{
  if (!std::isfinite(stamp) || stamp <= 0.0) {
    return;   // old format without a stamp: freshness unknown
  }
  if (have_phase && stamp < phase_stamp) {
    return;   // out of order (transient-local replay): keep the newest
  }
  have_phase = true;
  phase_stamp = stamp;
  paused = phasePausesReplan(p);
  path_blocked = p == ZonePhase::Blocked;
  if (phaseBlocksRecovery(p)) {
    have_block = true;
    block_stamp = stamp;
  }
}

bool PhaseMemory::fresh(double now, double timeout) const
{
  return have_phase && std::isfinite(now) && now >= phase_stamp && now - phase_stamp <= timeout;
}

bool PhaseMemory::pausedNow(double now, double timeout) const
{
  return fresh(now, timeout) && paused;
}

bool PhaseMemory::blockedRecently(double now, double timeout) const
{
  return have_block && std::isfinite(now) && now >= block_stamp && now - block_stamp <= timeout;
}

}  // namespace nav_behaviors
