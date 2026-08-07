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

  // IMU initialization

  // If IMU initialized
  //   IMU
  //   IMU forward propagation, for state and cov to image time.

  //   Lidar
  //   Lidar backward propagation, (deskew points based on image time)
  //   Lidar preprocessing + downsampling
  //   Lidar update (point-to-plane residual against map, iterate in iekf)

  //   Camera
  //   Undistort image
  //   Visual update (Sparse direct photometric update against visual points in
  //   map)

  //   Map update
  //   Update map with deskewed points in world frame
  //   Update visual map in map (new patches)
  //   Map sliding/culling if moved far enough
}

}  // namespace livo_slam
