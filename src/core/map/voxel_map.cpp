#include "voxel_map.h"

#include <cmath>
#include <unordered_set>

#include "core/math/voxel_key.h"

namespace livo_slam {

VoxelMap::VoxelMap(const VoxelMapParams& params) : params_(params) {}

void VoxelMap::InsertPoints(const std::vector<Point>& new_points) {
  for (const Point& point : new_points) {
    const Eigen::Vector3i voxel_index =
        voxel::VoxelIndex(point.position, params_.voxel_size);
    const auto flat_index = voxel::PackKey(voxel_index);

    const Eigen::Vector3d center =
        (voxel_index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
        params_.voxel_size;

    auto [it, inserted] =
        voxels_.try_emplace(flat_index, center, params_.voxel_size / 4.0, 0);

    it->second.InsertPoint(point, params_);
  }

  for (auto& [_key, voxel] : voxels_) {
    voxel.Update(params_);
  }
}

void VoxelMap::InsertVisualPoints(
    const std::vector<VisualPoint>& new_points,
    const std::vector<std::pair<Eigen::Vector3d, VisualPatch>>& new_patches,
    std::size_t max_patches_per_point) {
  // Insert new visual points
  for (const VisualPoint& point : new_points) {
    const auto voxel_it = voxels_.find(
        voxel::PackKey(voxel::VoxelIndex(point.position, params_.voxel_size)));
    if (voxel_it == voxels_.cend()) {
      continue;
    }
    voxel_it->second.InsertVisualPoint(point);
  }

  // Add new patches to existing visual points
  for (const auto& [position, patch] : new_patches) {
    const auto voxel_it = voxels_.find(
        voxel::PackKey(voxel::VoxelIndex(position, params_.voxel_size)));
    if (voxel_it == voxels_.cend()) {
      continue;
    }
    voxel_it->second.AddVisualPatch(position, patch, max_patches_per_point);
  }
}

std::vector<Plane> VoxelMap::CollectPlanes() const {
  std::vector<Plane> planes;
  planes.reserve(voxels_.size());
  for (const auto& [_key, voxel] : voxels_) {
    voxel.CollectPlanes(planes);
  }
  return planes;
}

std::optional<Plane> VoxelMap::PlaneAt(const Eigen::Vector3i& voxel_index,
                                       const Eigen::Vector3d& point) const {
  const auto voxel_it = voxels_.find(voxel::PackKey(voxel_index));
  if (voxel_it == voxels_.cend()) {
    return std::nullopt;
  }

  const VoxelOctree* correspondence =
      voxel_it->second.FindCorrespondence(point);
  if (correspondence == nullptr) {
    return std::nullopt;
  }

  const Plane& plane =
      *correspondence
           ->plane();  // SAFETY: Plane must exist if correspondence was found
  if (std::abs(plane.normal.dot(point - plane.centroid)) >=
      params_.max_match_distance) {
    return std::nullopt;
  }
  return plane;
}

std::optional<Plane> VoxelMap::FindPlane(const Eigen::Vector3d& point) const {
  // Check the voxel the point lands in.
  const Eigen::Vector3i voxel_index =
      voxel::VoxelIndex(point, params_.voxel_size);
  if (const auto plane = PlaneAt(voxel_index, point)) {
    return plane;
  }

  // Check the neighbor voxel on any axis where the point is more than a
  // quarter of the voxel size from the center.
  const Eigen::Vector3d center =
      (voxel_index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
      params_.voxel_size;
  const double quarter = params_.voxel_size / 4.0;

  // Find closest neighbor candidate
  // (can be original voxel if closer than quarter voxel size from center).
  Eigen::Vector3i neighbor_index = voxel_index;
  for (int axis = 0; axis < 3; ++axis) {
    if (point(axis) > center(axis) + quarter) {
      neighbor_index(axis) += 1;
    } else if (point(axis) < center(axis) - quarter) {
      neighbor_index(axis) -= 1;
    }
  }

  if (neighbor_index == voxel_index) {
    // No neighbor close enough.
    return std::nullopt;
  }
  return PlaneAt(neighbor_index, point);
}

std::vector<const VisualPoint*> VoxelMap::CollectVisualPoints(
    const std::vector<Point>& points) const {
  std::vector<const VisualPoint*> visual_points;

  std::unordered_set<std::int64_t> seen;
  for (const auto& point : points) {
    const auto key =
        voxel::PackKey(voxel::VoxelIndex(point.position, params_.voxel_size));
    if (!seen.insert(key).second) continue;

    if (const auto voxel = voxels_.find(key); voxel != voxels_.end()) {
      voxel->second.CollectVisualPoints(visual_points);
    }
  }
  return visual_points;
}

}  // namespace livo_slam
