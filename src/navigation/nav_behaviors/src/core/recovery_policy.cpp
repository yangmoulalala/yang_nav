#include "nav_behaviors/recovery_policy.hpp"

namespace nav_behaviors
{

RecoveryAction recoveryAction(const RecoveryConfig & c, int attempt, bool in_zone)
{
  const int retries = c.retries < 0 ? 0 : c.retries;
  if (attempt >= retries) {
    return RecoveryAction::GiveUp;
  }
  // inside a zone stretch the body has centimetre margins and the heading is locked: no spin, no
  // back-up.  Only drop the costmaps and let the controller continue on the shaped path.
  if (in_zone) {
    return RecoveryAction::ClearCostmaps;
  }
  if (c.spin && attempt >= c.spin_after && attempt < c.backup_after) {
    return RecoveryAction::Spin;
  }
  if (c.backup && attempt >= c.backup_after) {
    return RecoveryAction::BackUp;
  }
  return RecoveryAction::ClearCostmaps;
}

}  // namespace nav_behaviors
