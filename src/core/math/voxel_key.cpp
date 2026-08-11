#include "voxel_key.h"

namespace livo_slam::voxel {

Eigen::Vector3i VoxelIndex(const Eigen::Vector3d& position, double voxel_size) {
  return (position / voxel_size).array().floor().cast<int>();
}

// 21 bits per axis, biased to non-negative, so +-2^20 voxels per axis.
std::int64_t PackKey(const Eigen::Vector3i& index) {
  constexpr std::int64_t kBias = 1 << 20;
  constexpr std::int64_t kMask = 0x1FFFFF;

  return ((index.x() + kBias) & kMask) | (((index.y() + kBias) & kMask) << 21) |
         (((index.z() + kBias) & kMask) << 42);
}

}  // namespace livo_slam::voxel
