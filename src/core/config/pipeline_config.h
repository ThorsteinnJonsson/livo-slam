#pragma once

#include "core/config/camera_params.h"
#include "core/config/init_params.h"
#include "core/config/initial_state_params.h"
#include "core/config/lidar_params.h"
#include "core/config/map_params.h"
#include "core/config/process_noise_params.h"
#include "core/config/update_params.h"
#include "core/config/viz_params.h"

namespace livo_slam {

struct PipelineConfig {
  ImuInitParams init;
  InitialStateParams initial;
  ProcessNoiseParams process_noise;
  LidarParams lidar;
  CameraParams camera;
  VoxelMapParams map;
  VisualMapParams visual_map;
  LidarUpdateParams lidar_update;
  CameraUpdateParams camera_update;
  VizParams viz;
};

}  // namespace livo_slam
