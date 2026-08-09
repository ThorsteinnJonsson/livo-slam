#pragma once

#include "core/config/imu_noise_params.h"
#include "core/config/init_params.h"
#include "core/config/initial_state_params.h"
#include "core/config/lidar_params.h"
#include "core/config/viz_params.h"

namespace livo_slam {

struct PipelineConfig {
  ImuInitParams init;
  InitialStateParams initial;
  ImuNoiseParams imu_noise;
  LidarParams lidar;
  VizParams viz;
};

}  // namespace livo_slam
