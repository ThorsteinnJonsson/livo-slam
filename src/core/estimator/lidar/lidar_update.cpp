#include "lidar_update.h"

#include <cstddef>
#include <print>

#include "core/estimator/iekf.h"
#include "core/estimator/lidar/lidar_residual.h"
#include "core/types/state.h"

namespace livo_slam {

Estimate UpdateWithLidar(const Estimate& prior, const Cloud& cloud_imu,
                         const VoxelMap& map, const LidarUpdateParams& params) {
  Estimate posterior = prior;

  for (std::size_t iter = 0; iter < params.max_iterations; ++iter) {
    // Find matches
    const auto matches = BuildLidarMatches(cloud_imu, posterior.state, map);
    if (matches.size() < LidarUpdateParams::kMinMatches) {
      std::println("Too few matches, skipping lidar update.");
      return posterior;
    }

    IekfSystem system =
        BuildLidarSystem(matches, posterior.state, params.point_noise_std);

    auto iekf_result = SolveIekfStep(prior, posterior.state, system);

    posterior.state = BoxPlus(posterior.state, iekf_result.increment);
    posterior.covariance = iekf_result.covariance;

    if (IekfConverged(iekf_result.increment, params.rotation_epsilon,
                      params.position_epsilon)) {
      return posterior;
    }
  }

  return posterior;
}

}  // namespace livo_slam
