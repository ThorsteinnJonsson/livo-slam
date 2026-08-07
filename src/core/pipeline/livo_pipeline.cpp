#include "core/pipeline/livo_pipeline.h"

#include <print>

namespace livo_slam {

LivoPipeline::LivoPipeline(PipelineConfig config)
    : config_(config), imu_initializer_(config_.init) {}

void LivoPipeline::Process(const MeasureGroup& group) {
  const auto& [image, cloud, imu] = group;

  switch (imu_initializer_.status()) {
    case InitStatus::kCollecting:
    case InitStatus::kNotStationary: {
      imu_initializer_.Add(imu);
      if (imu_initializer_.status() == InitStatus::kInitialized) {
        const auto init_result = *imu_initializer_.init_result();
        estimate_.state.rotation = init_result.rotation;
        estimate_.state.gravity = init_result.gravity;
        estimate_.state.gyro_bias = init_result.gyro_bias;
        estimate_.timestamp = init_result.timestamp;
        std::println(
            "imu initialized t={:.4f} |a|={:.4f} bg=[{:.5f} {:.5f} {:.5f}]",
            init_result.timestamp, init_result.accel_mean_norm,
            init_result.gyro_bias.x(), init_result.gyro_bias.y(),
            init_result.gyro_bias.z());
      }
      return;
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
