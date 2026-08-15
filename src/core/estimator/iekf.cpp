#include "iekf.h"

#include "core/types/state.h"

namespace livo_slam {

IekfStepResult SolveIekfStep(const Estimate& prior, const State& current,
                             const IekfSystem& system) {
  const ErrorState offset = BoxMinus(prior.state, current);
  const ErrorMatrix posterior_cov =
      (system.hth + prior.covariance.inverse()).inverse();
  const ErrorMatrix gain = posterior_cov * system.hth;

  const ErrorState increment =
      posterior_cov * system.htz + (ErrorMatrix::Identity() - gain) * offset;

  return IekfStepResult{increment, posterior_cov};
}

bool IekfConverged(const ErrorState& increment, double rotation_epsilon,
                   double position_epsilon) {
  return increment.segment<3>(kIdxRotation).norm() < rotation_epsilon &&
         increment.segment<3>(kIdxPosition).norm() < position_epsilon;
}

}  // namespace livo_slam
