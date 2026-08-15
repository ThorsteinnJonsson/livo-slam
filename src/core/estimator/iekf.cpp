#include "iekf.h"

namespace livo_slam {

IekfStepResult SolveIekfStep(const Estimate& prior, const State& current,
                             const IekfSystem& system) {
  return IekfStepResult{ErrorState::Zero(), ErrorMatrix::Identity()};
}

bool IekfConverged(const ErrorState& increment, double rotation_epsilon,
                   double position_epsilon) {
  return false;
}

}  // namespace livo_slam
