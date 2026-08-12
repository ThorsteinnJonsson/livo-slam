#include "voxel_map.h"

#include "core/math/voxel_key.h"

namespace livo_slam {

VoxelMap::VoxelMap(const VoxelMapParams& params) : params_(params) {}

void VoxelMap::Insert(const Cloud& cloud) {
  for (const Point& point : cloud.points) {
    const Eigen::Vector3i voxel_index =
        voxel::VoxelIndex(point.position, params_.voxel_size);
    const auto flat_index = voxel::PackKey(voxel_index);

    const Eigen::Vector3d center =
        (voxel_index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
        params_.voxel_size;

    auto [it, inserted] =
        voxels_.try_emplace(flat_index, center, params_.voxel_size / 4.0, 0);

    it->second.Insert(point, params_);
  }

  for (auto& [_key, voxel] : voxels_) {
    voxel.Update(params_);
  }
}

std::optional<Plane> VoxelMap::FindPlane(const Eigen::Vector3d& point) const {
  // Find the voxel index and key for the point
  //
  // If that voxel exists
  //    FindCorrespond, and if it returns a node with a plane
  //       Accept it when |normal.(point - centroid)| < max_match_distance
  //
  // Otherwise try one neighbour voxel:
  //    Root center is (index + 0.5) * voxel_size, quarter is voxel_size / 4
  //    Per axis, shift the index by +1 when the point is past
  //    center + quarter, by -1 when it is before center - quarter
  //    If the shifted index differs, repeat the lookup and the test above
  //
  // Return nullopt

  const Eigen::Vector3i voxel_index =
      voxel::VoxelIndex(point.position, params_.voxel_size);
  const auto flat_index = voxel::PackKey(voxel_index);

  return std::nullopt;
}

}  // namespace livo_slam
