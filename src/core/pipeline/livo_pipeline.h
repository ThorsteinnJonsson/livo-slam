#pragma once

#include "core/config/pipeline_config.h"
#include "core/imu/imu_initializer.h"
#include "core/map/voxel_map.h"
#include "core/types/estimate.h"
#include "core/types/measure_group.h"

namespace livo_slam {

struct ProcessResult {
  Estimate estimate;
  Cloud cloud_world;
  Cloud cloud_deskewed_world;
  std::vector<Plane> map_planes;
};

class LivoPipeline {
 public:
  explicit LivoPipeline(PipelineConfig config);

  LivoPipeline(const LivoPipeline&) = delete;
  LivoPipeline& operator=(const LivoPipeline&) = delete;
  LivoPipeline(LivoPipeline&&) = delete;
  LivoPipeline& operator=(LivoPipeline&&) = delete;

  ProcessResult Process(const MeasureGroup& group);

 private:
  PipelineConfig config_;
  ImuInitializer imu_initializer_;
  VoxelMap local_map_;
  Estimate estimate_;
};

}  // namespace livo_slam
