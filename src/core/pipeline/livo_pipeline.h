#pragma once

#include "core/config/pipeline_config.h"
#include "core/imu/imu_initializer.h"
#include "core/types/measure_group.h"

namespace livo_slam {

class LivoPipeline {
 public:
  explicit LivoPipeline(PipelineConfig config);

  LivoPipeline(const LivoPipeline&) = delete;
  LivoPipeline& operator=(const LivoPipeline&) = delete;
  LivoPipeline(LivoPipeline&&) = delete;
  LivoPipeline& operator=(LivoPipeline&&) = delete;

  void Process(const MeasureGroup& group);

 private:
  PipelineConfig config_;
  ImuInitializer imu_initializer_;
};

}  // namespace livo_slam
