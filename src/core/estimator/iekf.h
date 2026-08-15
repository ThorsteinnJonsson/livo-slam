#pragma once

#include "core/types/estimate.h"
#include "core/types/state.h"

namespace livo_slam {

struct IekfSystem {
  ErrorMatrix hth = ErrorMatrix::Zero();  // H^T R^-1 H
  ErrorState htz = ErrorState::Zero();    // H^T R^-1 z
};

struct IekfStepResult {
  ErrorState increment = ErrorState::Zero();
  ErrorMatrix covariance = ErrorMatrix::Zero();
};

IekfStepResult SolveIekfStep(const Estimate& prior, const State& current,
                             const IekfSystem& system);

bool IekfConverged(const ErrorState& increment, double rotation_epsilon,
                   double position_epsilon);

}  // namespace livo_slam
