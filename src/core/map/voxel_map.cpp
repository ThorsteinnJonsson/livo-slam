#include "voxel_map.h"

#include "core/math/voxel_key.h"

namespace livo_slam {

VoxelMap::VoxelMap(const VoxelMapParams& params) : params_(params) {}

void VoxelMap::Insert(const Cloud& cloud_world) {
  for (const Point& point : cloud_world.points) {
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

}  // namespace livo_slam
