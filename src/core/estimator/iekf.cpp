#include "iekf.h"

namespace livo_slam {

ErrorState SolveIekfStep(const Estimate& prior, const State& current,
                         const IekfSystem& system, ErrorMatrix& covariance) {
  return ErrorState::Zero();
}

bool IekfConverged(const ErrorState& increment, double rotation_epsilon,
                   double position_epsilon) {
  return false;
}

}  // namespace livo_slam
