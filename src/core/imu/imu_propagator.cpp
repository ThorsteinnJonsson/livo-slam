#include "core/imu/imu_propagator.h"

#include <Eigen/Core>

#include "core/math/skew.h"

namespace livo_slam {

PropagationResult Propagate(const Estimate& estimate,
                            std::span<const ImuSample> imu_samples,
                            double end_time, const ImuNoiseParams& noise) {
  PropagationResult result;

  if (imu_samples.empty()) {
    result.estimate = estimate;
    result.estimate.timestamp = end_time;
    return result;
  }

  Estimate current = estimate;  // Explicit copy, we propagate this

  // We go over count+1 intervals:
  //   [estimate.timestamp, s0], each [si, si+1], and [s_last, end_time]
  const size_t count = imu_samples.size();
  for (size_t i = 0; i <= count; ++i) {
    auto& [state, cov, timestamp] = current;

    // IMU samples
    const auto& m0 = imu_samples[(i == 0) ? 0 : i - 1];
    const auto& m1 = imu_samples[(i == count) ? count - 1 : i];
    const double t1 = (i == count) ? end_time : imu_samples[i].timestamp;
    const double dt = t1 - timestamp;
    if (dt <= 0.0) {
      continue;
    }

    // Bias-corrected midpoint
    const Eigen::Vector3d w0 = m0.angular_velocity - state.gyro_bias;
    const Eigen::Vector3d w1 = m1.angular_velocity - state.gyro_bias;
    const Eigen::Vector3d a0 = m0.linear_acceleration - state.accel_bias;
    const Eigen::Vector3d a1 = m1.linear_acceleration - state.accel_bias;
    const Eigen::Vector3d w_mid = 0.5 * (w0 + w1);
    const Eigen::Vector3d a_mid = 0.5 * (a0 + a1);
    const Sophus::SO3d R_mid =
        state.rotation * Sophus::SO3d::exp(0.5 * w_mid * dt);

    // ──────────────────────────────────────────────────────────────
    // Record the pose and IMU measurement the interval starts from.
    // This will be used for backward propagation (lidar deskewing) later.
    ImuPose pose;
    pose.timestamp = timestamp;
    pose.pose = Sophus::SE3d(state.rotation, state.position);
    pose.velocity = state.velocity;
    pose.angular_velocity = w_mid;
    pose.linear_acceleration = a_mid;
    result.trajectory.push_back(pose);

    // ──────────────────────────────────────────────────────────────
    // Propagate new state
    State new_state;
    const Eigen::Vector3d accel_world = R_mid * a_mid + state.gravity;
    new_state.rotation = state.rotation * Sophus::SO3d::exp(w_mid * dt);
    new_state.position =
        state.position + state.velocity * dt + 0.5 * accel_world * dt * dt;
    new_state.velocity = state.velocity + accel_world * dt;
    new_state.gravity = state.gravity;
    new_state.gyro_bias = state.gyro_bias;
    new_state.accel_bias = state.accel_bias;
    new_state.inv_exposure = state.inv_exposure;

    // ──────────────────────────────────────────────────────────────
    // Propagate new covariance

    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d R_mat = state.rotation.matrix();

    // ── F_c (continuous error-state transition Jacobian)
    ErrorMatrix Fc = ErrorMatrix::Zero();

    // δθ row
    Fc.block<3, 3>(0, 0) = -Skew(w_mid);  // -[ω̂]×
    Fc.block<3, 3>(0, 9) = -I3;           // -δb_g

    // δp row
    Fc.block<3, 3>(3, 6) = I3;  // δv

    // δv row
    Fc.block<3, 3>(6, 0) = -R_mat * Skew(a_mid);  // -R[â]×
    Fc.block<3, 3>(6, 12) = -R_mat;               // -R δb_a
    // a_world = R·â + g, so ∂(a_world)/∂δg is the identity (gravity is R³).
    Fc.block<3, 3>(6, 15) = I3;

    // ── F_d ≈ I + F_c·dt
    // (discrete error-state Jacobian, first-order approximation)
    const ErrorMatrix Fd = ErrorMatrix::Identity() + Fc * dt;

    // ── Additive noise covariance Q_d = G_c·Q_c·G_c^T·dt
    // G_c·Q_c·G_c^T is block-diagonal with blocks:
    //   σ_g²·I   (δθ)
    //   0        (δp)
    //   σ_a²·I   (δv,  since R*I*R^T = I)
    //   σ_bg²·I  (δb_g)
    //   σ_ba²·I  (δb_a)
    //   0        (δg)
    const double sg2 = noise.gyro_noise_std * noise.gyro_noise_std;
    const double sa2 = noise.accel_noise_std * noise.accel_noise_std;
    const double sbg2 = noise.gyro_random_walk_std * noise.gyro_random_walk_std;
    const double sba2 =
        noise.accel_random_walk_std * noise.accel_random_walk_std;

    ErrorMatrix Qd = ErrorMatrix::Zero();
    Qd.block<3, 3>(0, 0) = sg2 * dt * I3;
    Qd.block<3, 3>(6, 6) = sa2 * dt * I3;
    Qd.block<3, 3>(9, 9) = sbg2 * dt * I3;
    Qd.block<3, 3>(12, 12) = sba2 * dt * I3;

    const ErrorMatrix new_cov = Fd * cov * Fd.transpose() + Qd;

    state = new_state;
    cov = new_cov;
    timestamp = t1;
  }

  current.timestamp = end_time;
  result.estimate = current;
  return result;
}

}  // namespace livo_slam
