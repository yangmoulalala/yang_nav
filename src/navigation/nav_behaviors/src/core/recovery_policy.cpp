#include "nav_behaviors/recovery_policy.hpp"

#include <algorithm>

namespace nav_behaviors
{

RecoveryAction recoveryAction(const RecoveryConfig & c, std::uint64_t attempt, bool in_zone)
{
  const int retries = c.retries < 0 ? 0 : c.retries;
  if (c.retries != -1 && attempt >= static_cast<std::uint64_t>(retries)) {
    return RecoveryAction::GiveUp;
  }
  // inside a zone stretch the body has centimetre margins and the heading is locked: no spin, no
  // back-up.  Only drop the costmaps and let the controller continue on the shaped path.
  if (in_zone) {
    return RecoveryAction::ClearCostmaps;
  }
  // Unlimited recovery must revisit clearing and observing instead of backing up forever.
  const auto cycle = static_cast<std::uint64_t>(std::max({1, c.spin_after, c.backup_after})) + 1;
  const auto stage = c.retries == -1 ? attempt % cycle : attempt;
  const auto spin_after = static_cast<std::uint64_t>(std::max(0, c.spin_after));
  const auto backup_after = static_cast<std::uint64_t>(std::max(0, c.backup_after));
  if (c.spin && stage >= spin_after && stage < backup_after) {
    return RecoveryAction::Spin;
  }
  if (c.backup && stage >= backup_after) {
    return RecoveryAction::BackUp;
  }
  return RecoveryAction::ClearCostmaps;
}

}  // namespace nav_behaviors
