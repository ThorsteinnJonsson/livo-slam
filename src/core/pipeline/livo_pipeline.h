#pragma once

#include "core/imu/imu_initializer.h"
#include "core/types/measure_group.h"

namespace livo_slam {

class LivoPipeline {
 public:
  LivoPipeline();

  LivoPipeline(const LivoPipeline&) = delete;
  LivoPipeline& operator=(const LivoPipeline&) = delete;
  LivoPipeline(LivoPipeline&&) = delete;
  LivoPipeline& operator=(LivoPipeline&&) = delete;

  void Process(const MeasureGroup& group);

 private:
  ImuInitializer imu_initializer_;
  //   enum ImuInitialization {
  //     kNotInitialized,
  //     kInProgress,
  //     kInitialized,
  //   };
  //   ImuInitialization imu_initialization_state_ =
  //   ImuInitialization::kInitialized;
};

}  // namespace livo_slam
