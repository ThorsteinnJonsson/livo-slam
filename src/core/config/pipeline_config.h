#pragma once

#include "core/config/imu_noise_params.h"
#include "core/config/init_params.h"
#include "core/config/initial_state_params.h"

namespace livo_slam {

struct PipelineConfig {
  ImuInitParams init;
  InitialStateParams initial;
  ImuNoiseParams imu_noise;
};

}  // namespace livo_slam
