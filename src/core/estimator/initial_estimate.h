#pragma once

#include "core/config/initial_state_params.h"
#include "core/imu/imu_initializer.h"
#include "core/types/estimate.h"

namespace livo_slam {

Estimate MakeInitialEstimate(const InitResult& result,
                             const InitialStateParams& params);

}  // namespace livo_slam
