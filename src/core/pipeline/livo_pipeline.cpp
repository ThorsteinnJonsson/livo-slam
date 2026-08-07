#include "core/pipeline/livo_pipeline.h"

#include <print>

namespace livo_slam {

LivoPipeline::LivoPipeline() = default;

void LivoPipeline::Process(const MeasureGroup& group) {
  const auto& [image, cloud, imu] = group;

  std::println("imu={} | image={}x{} {} t={:.4f} | points={} cloud span={:.4f}",
               imu.size(), image.width, image.height, image.encoding,
               image.timestamp, cloud.points.size(),
               cloud.end_time - cloud.start_time);
}

}  // namespace livo_slam
