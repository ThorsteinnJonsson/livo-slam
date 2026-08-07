#include "core/pipeline/livo_pipeline.h"

#include <print>

namespace livo_slam {

// TODO: Get initializer params from file
LivoPipeline::LivoPipeline() : imu_initializer_(InitParams{}) {}

void LivoPipeline::Process(const MeasureGroup& group) {
  const auto& [image, cloud, imu] = group;

  switch (imu_initializer_.status()) {
    case InitStatus::kCollecting: {
      const InitResult result = imu_initializer_.Add(imu);
      if (result.status != InitStatus::kInitialized) {
        return;
      }
      initial_ = result;
      std::println(
          "imu initialized t={:.4f} |a|={:.4f} bg=[{:.5f} {:.5f} {:.5f}]",
          result.timestamp, result.accel_mean_norm, result.gyro_bias.x(),
          result.gyro_bias.y(), result.gyro_bias.z());
      break;
    }
    case InitStatus::kNotStationary: {
      break;
    }
    case InitStatus::kInitialized: {
      std::println(
          "imu={} | image={}x{} {} t={:.4f} | points={} cloud span={:.4f}",
          imu.size(), image.width, image.height, image.encoding,
          image.timestamp, cloud.points.size(),
          cloud.end_time - cloud.start_time);
      break;
    }
  }

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
