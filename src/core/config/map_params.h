#pragma once

#include <array>
#include <cstddef>

namespace livo_slam {

struct VoxelMapParams {
  // Each voxel can be subdivided like an octree to kMaxLayerDepth
  static constexpr std::size_t kMaxLayerDepth = 3;

  double voxel_size = 0.5;
  std::array<std::size_t, kMaxLayerDepth> layer_init_points = {5, 5, 5};
  std::size_t max_points_per_voxel = 50;
  double max_plane_variance = 0.01;
  double max_match_distance = 0.5;
};

struct VisualMapParams {
  std::size_t grid_size = 30;
  double min_corner_score = 50.0;
  double min_seconds_between_patches = 2.0;
  double min_patch_pixel_distance = 40.0;
};

}  // namespace livo_slam
