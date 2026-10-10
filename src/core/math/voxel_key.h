#pragma once

#include <Eigen/Core>
#include <cstdint>

namespace livo_slam::voxel {

Eigen::Vector3i VoxelIndex(const Eigen::Vector3d& position, double voxel_size);

std::int64_t PackKey(const Eigen::Vector3i& index);

Eigen::Vector3i UnpackKey(std::int64_t key);

}  // namespace livo_slam::voxel
