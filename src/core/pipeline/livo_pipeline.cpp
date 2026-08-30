#include "core/pipeline/livo_pipeline.h"

#include <opencv2/imgproc.hpp>
#include <print>

#include "core/estimator/camera/camera_update.h"
#include "core/estimator/initial_estimate.h"
#include "core/estimator/lidar/lidar_update.h"
#include "core/imu/imu_propagator.h"
#include "core/lidar/deskew.h"
#include "core/lidar/downsample.h"
#include "core/lidar/range_filter.h"
#include "core/map/camera/visual_point_selection.h"
#include "core/types/visual_point.h"

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
      const Cloud deskewed = Deskew(in_range, propagation_result.trajectory,
                                    estimate_.state, config_.lidar.T_imu_lidar);
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

      // Build image pyramid
      std::vector<cv::Mat> image_pyramid;
      static_assert(CameraUpdateParams::kPyramidLevels > 0,
                    "Pyramid level needs to be larger than zero.");
      constexpr int kMaxLevel =  // Minus one because 0-based index
          static_cast<int>(CameraUpdateParams::kPyramidLevels - 1);
      cv::buildPyramid(undistorted_image.image, image_pyramid, kMaxLevel);

      // Collect candidate visual points
      std::vector<Point> points_world = downsampled.points;
      {
        const Sophus::SE3d T_world_imu(estimate_.state.rotation,
                                       estimate_.state.position);
        for (Point& point : points_world) {
          point.position = T_world_imu * point.position;
        }
      }
      const std::vector<const VisualPoint*> visual_points =
          local_map_.CollectVisualPoints(points_world);

      //   Visual update (Sparse direct photometric update against visual
      //   points in map)
      estimate_ = UpdateWithCamera(estimate_, image_pyramid, visual_points,
                                   config_.camera, config_.camera_update);

      //   Map update
      //   Update map with deskewed points in world frame

      std::vector<Point> points_deskewed_world = downsampled.points;
      {
        const Sophus::SE3d T_world_imu(estimate_.state.rotation,
                                       estimate_.state.position);
        for (Point& point : points_deskewed_world) {
          point.position = T_world_imu * point.position;
        }
      }

      local_map_.InsertPoints(points_deskewed_world);

      //   Add new visual points in map and update existing ones with new
      //   patches
      const auto& [new_visual_points, new_visual_patches] = SelectVisualPoints(
          points_deskewed_world, image_pyramid, image.timestamp,
          estimate_.state, config_.camera, config_.camera_update,
          config_.visual_map, visual_points);
      local_map_.InsertVisualPoints(new_visual_points, new_visual_patches);

      //   Map sliding/culling if moved far enough
      //     TODO

      // Process output for visualization
      const Sophus::SE3d T_world_imu(estimate_.state.rotation,
                                     estimate_.state.position);
      const Sophus::SE3d T_world_lidar =
          T_world_imu * config_.lidar.T_imu_lidar;
      Cloud cloud_world = cloud;
      for (Point& point : cloud_world.points) {
        point.position = T_world_lidar * point.position;
      }

      return {estimate_, std::move(cloud_world), local_map_.CollectPlanes()};
    }
  }

  return {estimate_, {}, {}};
}

}  // namespace livo_slam
