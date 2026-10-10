#include "voxel_octree.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

#include "core/config/map_params.h"
#include "core/types/visual_point.h"

namespace livo_slam {

VoxelOctree::VoxelOctree(const Eigen::Vector3d& center, double quarter_length,
                         std::size_t layer)
    : center_(center), quarter_length_(quarter_length), layer_(layer) {}

void VoxelOctree::InsertPoint(const Point& point,
                              const VoxelMapParams& params) {
  if (!update_enabled_) return;

  if (IsSubdivided()) {
    const auto child_index = ChildIndex(point.position);
    VoxelOctree& child = GetChild(child_index);
    child.InsertPoint(point, params);
  } else if (points_.size() < params.max_points_per_voxel) {
    points_.push_back(point);
    ++new_points_;
  }
}

void VoxelOctree::InsertVisualPoint(const VisualPoint& visual_point) {
  visual_points_.push_back(visual_point);
}

void VoxelOctree::AddVisualPatch(const Eigen::Vector3d& position,
                                 const VisualPatch& visual_patch,
                                 std::size_t max_patches) {
  auto found = std::find_if(
      visual_points_.begin(), visual_points_.end(),
      [&](const VisualPoint& p) { return p.position == position; });
  if (found == visual_points_.end()) {
    return;
  }

  found->patches.push_back(visual_patch);
  if (max_patches == 0 || found->patches.size() <= max_patches) {
    return;
  }

  // How square on the camera was to the surface when the patch was taken.
  const auto viewed_head_on = [&found](const VisualPatch& patch) {
    const Eigen::Vector3d view =
        found->position - patch.T_world_camera.translation();
    const double range = view.norm();
    if (range <= 0.0) return 0.0;
    return std::abs(found->normal.dot(view / range));
  };

  const auto worst = std::min_element(
      found->patches.begin(), found->patches.end(),
      [&viewed_head_on](const VisualPatch& a, const VisualPatch& b) {
        return viewed_head_on(a) < viewed_head_on(b);
      });
  found->patches.erase(worst);
}

void VoxelOctree::Update(const VoxelMapParams& params) {
  if (IsSubdivided()) {
    for (const auto& child : children_) {
      if (child) {
        child->Update(params);
      }
    }
    return;
  }

  if (new_points_ == 0) {
    // Early return, nothing to update
    return;
  }

  if (!initialized_ && points_.size() > params.layer_init_points[layer_]) {
    InitPlane(params);
    initialized_ = true;
  } else if (plane_) {
    InitPlane(params);  // Refit with new points
  }

  if (initialized_ && !plane_ && layer_ + 1 < VoxelMapParams::kMaxLayerDepth) {
    Subdivide(params);
    for (const auto& child : children_) {
      if (child) {
        child->Update(params);
      }
    }
  }

  if (points_.size() >= params.max_points_per_voxel) {
    update_enabled_ = false;
    if (plane_) {
      points_.clear();
    }
  }

  new_points_ = 0;
}

void VoxelOctree::InitPlane(const VoxelMapParams& params) {
  plane_.reset();
  if (points_.size() < 3) {
    return;
  }

  const double count = static_cast<double>(points_.size());

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Point& point : points_) {
    centroid += point.position;
  }
  centroid /= count;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Point& point : points_) {
    const Eigen::Vector3d offset = point.position - centroid;
    covariance += offset * offset.transpose();
  }
  covariance /= count;

  // Eigenvalues come out in increasing order, so the first one is the
  // mean squared distance to the best-fit plane.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  const double variance = solver.eigenvalues()(0);
  if (variance > params.max_plane_variance) {
    return;
  }

  Plane plane;
  plane.centroid = centroid;
  plane.normal = solver.eigenvectors().col(0);
  plane.variance = variance;
  plane_ = plane;
}

void VoxelOctree::Subdivide(const VoxelMapParams& params) {
  for (const Point& point : points_) {
    const auto child_index = ChildIndex(point.position);
    VoxelOctree& child = GetChild(child_index);
    child.InsertPoint(point, params);
  }
  points_.clear();
}

const VoxelOctree* VoxelOctree::FindCorrespondence(
    const Eigen::Vector3d& point) const {
  if (plane_) return this;

  if (!IsSubdivided()) return nullptr;

  if (const VoxelOctree* child = children_[ChildIndex(point)].get();
      child != nullptr) {
    return child->FindCorrespondence(point);
  }
  return nullptr;
}

void VoxelOctree::CollectVisualPoints(
    std::vector<const VisualPoint*>& visual_points) const {
  // Push back const pointer to own visual points
  for (const VisualPoint& p : visual_points_) {
    visual_points.push_back(&p);
  }

  // Recuse into children
  for (const auto& child : children_) {
    if (child) child->CollectVisualPoints(visual_points);
  }
}

void VoxelOctree::CollectPlanes(std::vector<Plane>& planes) const {
  if (plane_) {
    planes.push_back(*plane_);
    return;
  }
  for (const auto& child : children_) {
    if (child) {
      child->CollectPlanes(planes);
    }
  }
}

std::size_t VoxelOctree::ChildIndex(const Eigen::Vector3d& point) const {
  const std::size_t x = point.x() > center_.x() ? 1 : 0;
  const std::size_t y = point.y() > center_.y() ? 1 : 0;
  const std::size_t z = point.z() > center_.z() ? 1 : 0;
  return 4 * x + 2 * y + z;
}

VoxelOctree& VoxelOctree::GetChild(std::size_t index) {
  auto& child = children_[index];
  if (!child) {
    const Eigen::Vector3d offset((index & 4 ? 1.0 : -1.0) * quarter_length_,
                                 (index & 2 ? 1.0 : -1.0) * quarter_length_,
                                 (index & 1 ? 1.0 : -1.0) * quarter_length_);
    child = std::make_unique<VoxelOctree>(center_ + offset,
                                          quarter_length_ / 2.0, layer_ + 1);
  }
  return *child;
}

bool VoxelOctree::IsSubdivided() const {
  return initialized_ && !plane_ && layer_ + 1 < VoxelMapParams::kMaxLayerDepth;
}

}  // namespace livo_slam