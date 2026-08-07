#pragma once

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
};

}  // namespace livo_slam
