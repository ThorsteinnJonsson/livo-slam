#include "core/pipeline/livo_pipeline.h"

#include <print>

#include "core/estimator/camera/camera_update.h"
#include "core/estimator/initial_estimate.h"
#include "core/estimator/lidar/lidar_update.h"
#include "core/imu/imu_propagator.h"
#include "core/lidar/deskew.h"
#include "core/lidar/downsample.h"
#include "core/lidar/range_filter.h"

namespace livo_slam {

LivoPipeline::LivoPipeline(PipelineConfig config)
    : config_(config),
      imu_initializer_(config_.init),
      image_undistorter_(config_.camera),
      local_map_(config_.map) {}

ProcessResult LivoPipeline::Process(const MeasureGroup& group) {
  const auto& [image, cloud, imu_samples] = group;

  switch (imu_initializer_.status()) {
    case InitStatus::kCollecting:
    case InitStatus::kNotStationary: {
      imu_initializer_.Add(imu_samples);
      if (imu_initializer_.status() == InitStatus::kInitialized) {
        const auto init_result = *imu_initializer_.init_result();
        estimate_ = MakeInitialEstimate(init_result, config_.initial);
        std::println(
            "imu initialized t={:.4f} |a|={:.4f} bg=[{:.5f} {:.5f} {:.5f}]",
            init_result.timestamp, init_result.accel_mean_norm,
            init_result.gyro_bias.x(), init_result.gyro_bias.y(),
            init_result.gyro_bias.z());

        const State& state = estimate_.state;
        const Eigen::Vector3d ypr =
            state.rotation.matrix().eulerAngles(2, 1, 0) * 180.0 / M_PI;
        std::println("initial state t={:.4f}", estimate_.timestamp);
        std::println("  ypr deg    = [{:.3f} {:.3f} {:.3f}]", ypr.x(), ypr.y(),
                     ypr.z());
        std::println("  position   = [{:.4f} {:.4f} {:.4f}]",
                     state.position.x(), state.position.y(),
                     state.position.z());
        std::println("  velocity   = [{:.4f} {:.4f} {:.4f}]",
                     state.velocity.x(), state.velocity.y(),
                     state.velocity.z());
        std::println("  gyro_bias  = [{:.5f} {:.5f} {:.5f}]",
                     state.gyro_bias.x(), state.gyro_bias.y(),
                     state.gyro_bias.z());
        std::println("  accel_bias = [{:.5f} {:.5f} {:.5f}]",
                     state.accel_bias.x(), state.accel_bias.y(),
                     state.accel_bias.z());
        std::println("  gravity    = [{:.4f} {:.4f} {:.4f}]", state.gravity.x(),
                     state.gravity.y(), state.gravity.z());
        std::println("  cov diag   = [{:.2e} {:.2e} {:.2e} {:.2e} {:.2e}]",
                     estimate_.covariance(kIdxRotation, kIdxRotation),
                     estimate_.covariance(kIdxPosition, kIdxPosition),
                     estimate_.covariance(kIdxVelocity, kIdxVelocity),
                     estimate_.covariance(kIdxGyroBias, kIdxGyroBias),
                     estimate_.covariance(kIdxAccelBias, kIdxAccelBias));
      }
      return {estimate_, {}, {}};
    }
    case InitStatus::kInitialized: {
      std::println(
          "imu={} | image={}x{} t={:.4f} | points={} cloud span={:.4f}",
          imu_samples.size(), image.image.cols, image.image.rows,
          image.timestamp, cloud.points.size(),
          cloud.end_time - cloud.start_time);

      //   IMU
      //   IMU forward propagation, for state and cov to image time.
      if (imu_samples.size() < 2) {
        std::println("Too few IMU samples to propagate");
        return {estimate_, {}, {}};
      }
      const auto propagation_result = Propagate(
          estimate_, imu_samples, image.timestamp, config_.process_noise);
      estimate_ = propagation_result.estimate;

      std::println(
          "  propagated p=[{:.4f} {:.4f} {:.4f}] v=[{:.4f} {:.4f} {:.4f}]",
          estimate_.state.position.x(), estimate_.state.position.y(),
          estimate_.state.position.z(), estimate_.state.velocity.x(),
          estimate_.state.velocity.y(), estimate_.state.velocity.z());

      //   Lidar
      //   Lidar preprocessing
      const Cloud in_range = FilterByRange(cloud, config_.lidar.min_range,
                                           config_.lidar.max_range);

      //   Lidar backward propagation, (deskew points based on image time)
      const Cloud deskewed =
          Deskew(in_range, propagation_result.trajectory, estimate_.state,
                 config_.lidar.imu_from_lidar);
      //   Lidar downsampling
      const Cloud downsampled =
          VoxelDownsample(deskewed, config_.lidar.voxel_leaf_size);

      std::println("  points raw={} in_range={} downsampled={}",
                   cloud.points.size(), in_range.points.size(),
                   downsampled.points.size());

      //   Lidar update (point-to-plane residual against map, iterate in
      //   iekf)
      if (!local_map_.Empty()) {
        estimate_ = UpdateWithLidar(estimate_, downsampled, local_map_,
                                    config_.lidar_update);
      }

      //   Camera
      //   Undistort image
      const auto undistorted_image = image_undistorter_.Undistort(image);

      //   Visual update (Sparse direct photometric update against visual
      //   points in map)
      estimate_ = UpdateWithCamera(estimate_, undistorted_image, local_map_,
                                   config_.camera_update);

      //   Map update
      //   Update map with deskewed points in world frame
      //   Update visual map in map (new patches)
      //   Map sliding/culling if moved far enough
      const Sophus::SE3d T_world_imu(estimate_.state.rotation,
                                     estimate_.state.position);
      const Sophus::SE3d T_world_lidar =
          T_world_imu * config_.lidar.imu_from_lidar;

      Cloud cloud_world = cloud;
      for (Point& point : cloud_world.points) {
        point.position = T_world_lidar * point.position;
      }

      Cloud deskewed_world = downsampled;
      for (Point& point : deskewed_world.points) {
        point.position = T_world_imu * point.position;
      }

      local_map_.Insert(deskewed_world);

      return {estimate_, std::move(cloud_world), local_map_.CollectPlanes()};
    }
  }

  return {estimate_, {}, {}};
}

}  // namespace livo_slam
