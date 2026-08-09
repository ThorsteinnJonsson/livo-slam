#include "deskew.h"

#include <algorithm>

#include "core/types/trajectory.h"

namespace livo_slam {

namespace {
Sophus::SE3d InterpolatePose(const Trajectory& trajectory, double timestamp) {
  if (timestamp <= trajectory.front().timestamp) return trajectory.front().pose;
  if (timestamp >= trajectory.back().timestamp) return trajectory.back().pose;

  // First pose with timestamp strictly greater than the query.
  auto hi = std::upper_bound(
      trajectory.begin(), trajectory.end(), timestamp,
      [](double t, const ImuPose& p) { return t < p.timestamp; });
  auto lo = std::prev(hi);

  const double alpha =
      (timestamp - lo->timestamp) / (hi->timestamp - lo->timestamp);

  // SE3 interpolation via the Lie-algebra tangent between the two poses.
  const Sophus::SE3d rel = lo->pose.inverse() * hi->pose;
  return lo->pose * Sophus::SE3d::exp(alpha * rel.log());
}
}  // namespace

// Deskew pointcloud and output in IMU frame
Cloud Deskew(const Cloud& cloud, const Trajectory& trajectory,
             const State& reference_state, const Sophus::SE3d& T_imu_lidar) {
  Cloud deskewed = cloud;
  if (trajectory.empty()) {
    return deskewed;
  }

  const Sophus::SE3d T_W_imu_ref(reference_state.rotation,
                                 reference_state.position);
  const Sophus::SE3d T_W_imu_ref_inv = T_W_imu_ref.inverse();

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const double t_i = cloud.points[i].timestamp;
    const Sophus::SE3d T_W_imu_i = InterpolatePose(trajectory, t_i);

    // Relative body motion from the point's capture time to the reference time,
    // re-expressed in the IMU frame.
    const Sophus::SE3d correction = (T_W_imu_ref_inv * T_W_imu_i) * T_imu_lidar;

    const Eigen::Vector3d& p_i = cloud.points[i].position;
    const Eigen::Vector3d p_ref = correction * p_i;

    auto& p_out = deskewed.points[i].position;
    p_out = p_ref;
  }
  return deskewed;
}

}  // namespace livo_slam
