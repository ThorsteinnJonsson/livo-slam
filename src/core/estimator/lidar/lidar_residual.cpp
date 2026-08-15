#include "lidar_residual.h"

#include <sophus/se3.hpp>

namespace livo_slam {

std::vector<PointToPlaneMatch> BuildLidarMatches(const Cloud& cloud_imu,
                                                 const State& state,
                                                 const VoxelMap& map) {
  std::vector<PointToPlaneMatch> matches;

  const Sophus::SE3d T_world_imu{state.rotation, state.position};
  for (const Point& point_imu : cloud_imu.points) {
    const Eigen::Vector3d point_world = T_world_imu * point_imu.position;

    const auto plane = map.FindPlane(point_world);
    if (plane.has_value()) {
      const double distance = plane->normal.dot(point_world - plane->centroid);
      matches.push_back(
          PointToPlaneMatch{point_imu.position, plane->normal, distance});
    }
  }

  return matches;
}

IekfSystem BuildLidarSystem(const std::vector<PointToPlaneMatch>& matches,
                            const State& state, double point_noise_std) {
  static_assert(kIdxPosition == kIdxRotation + 3,
                "The 6x6 block below assumes rotation and position are "
                "contiguous in the error state");

  IekfSystem system;

  const double weight = 1.0 / (point_noise_std * point_noise_std);
  const Eigen::Matrix3d rotation_transpose = state.rotation.inverse().matrix();

  for (const auto& match : matches) {
    Eigen::Matrix<double, 6, 1> jacobian;
    jacobian.head<3>() =
        match.point_imu.cross(rotation_transpose * match.normal);
    jacobian.tail<3>() = match.normal;

    system.hth.block<6, 6>(kIdxRotation, kIdxRotation) +=
        weight * jacobian * jacobian.transpose();
    system.htz.segment<6>(kIdxRotation) += weight * jacobian * -match.distance;
  }

  return system;
}

}  // namespace livo_slam
