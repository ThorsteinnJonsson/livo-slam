#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace livo_slam {

struct InitialStateParams {
  Sophus::SO3d rotation_imu_lidar;
  Eigen::Vector3d position_imu_lidar = Eigen::Vector3d::Zero();

  Sophus::SO3d rotation_imu_camera;
  Eigen::Vector3d position_imu_camera = Eigen::Vector3d::Zero();

  double rotation_std = 0.017;
  double position_std = 0.001;
  double velocity_std = 0.01;
  double gyro_bias_std = 0.001;
  double accel_bias_std = 0.01;
  double gravity_std = 0.01;
  double lidar_rotation_std = 0.0035;
  double lidar_position_std = 0.002;
  double camera_rotation_std = 0.0035;
  double camera_position_std = 0.002;
};

}  // namespace livo_slam
