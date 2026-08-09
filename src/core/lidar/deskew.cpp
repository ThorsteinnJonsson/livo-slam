#include "deskew.h"

namespace livo_slam {

namespace {
Sophus::SE3d InterpolatePose(const Trajectory& trajectory, double t) {
  // TODO: Implement
  return {};
}
}  // namespace

Cloud Deskew(const Cloud& cloud, const Trajectory& trajectory,
             const State& reference_state, double reference_time,
             const Sophus::SE3d& T_imu_lidar) {
  Cloud deskewed = cloud;
  if (trajectory.poses.empty()) {
    return deskewed;
  }

  const Sophus::SE3d T_lidar_imu = T_imu_lidar.inverse();
  const Sophus::SE3d T_W_imu_ref = InterpolatePose(trajectory, reference_time);
  const Sophus::SE3d T_W_imu_ref_inv = T_W_imu_ref.inverse();

  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const double t_i = cloud.points[i].timestamp;
    const Sophus::SE3d T_W_imu_i = InterpolatePose(trajectory, t_i);

    // Relative body motion from the point's capture time to the reference time,
    // re-expressed in the lidar frame.
    const Sophus::SE3d correction =
        T_lidar_imu * (T_W_imu_ref_inv * T_W_imu_i) * T_imu_lidar;

    const Eigen::Vector3d& p_i = cloud.points[i].position;
    const Eigen::Vector3d p_ref = correction * p_i;

    auto& p_out = deskewed.points[i].position;
    p_out.x() = static_cast<float>(p_ref.x());
    p_out.y() = static_cast<float>(p_ref.y());
    p_out.z() = static_cast<float>(p_ref.z());
  }
  return deskewed;
}

}  // namespace livo_slam
