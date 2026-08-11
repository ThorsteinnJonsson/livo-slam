#include "voxel_octree.h"

namespace livo_slam {

VoxelOctree::VoxelOctree(const Eigen::Vector3d& center, double quarter_length,
                         std::size_t layer) {};

void VoxelOctree::Insert(const Point& point_world,
                         const VoxelMapParams& params) {}

void VoxelOctree::InitPlane() {}

void VoxelOctree::Subdivide() {}

int VoxelOctree::ChildIndex(const Eigen::Vector3d& point_world) const {}

}  // namespace livo_slam