#pragma once

#include <span>

#include "core/config/imu_noise_params.h"
#include "core/types/estimate.h"
#include "core/types/imu_sample.h"
#include "core/types/trajectory.h"

namespace livo_slam {

struct PropagationResult {
  Estimate estimate;
  Trajectory trajectory;
};

PropagationResult Propagate(const Estimate& estimate,
                            std::span<const ImuSample> samples, double end_time,
                            const ImuNoiseParams& noise);

}  // namespace livo_slam
