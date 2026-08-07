#pragma once

namespace livo_slam {

class LivoPipeline {
 public:
  LivoPipeline();

  LivoPipeline(const LivoPipeline&) = delete;
  LivoPipeline& operator=(const LivoPipeline&) = delete;
  LivoPipeline(LivoPipeline&&) = delete;
  LivoPipeline& operator=(LivoPipeline&&) = delete;
};

}  // namespace livo_slam