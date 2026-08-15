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

}  // namespace livo_slam
