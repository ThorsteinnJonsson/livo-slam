#include "voxel_map.h"

namespace livo_slam {

VoxelMap::VoxelMap(const VoxelMapParams& params)
    : voxel_size_(params.voxel_size),
      layer_init_points_(params.layer_init_points) {}

void VoxelMap::Insert(const Cloud& cloud_world) {}
}  // namespace livo_slam
