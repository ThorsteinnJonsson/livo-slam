#include "core/map/voxel_octree.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

VoxelMapParams MakeParams() {
  VoxelMapParams params;
  params.voxel_size = 1.0;
  params.layer_init_points = {5, 5, 5};
  params.max_points_per_voxel = 50;
  params.max_plane_variance = 0.01;
  return params;
}

// A root node spanning [0, 1] on every axis.
VoxelOctree MakeRoot() {
  return VoxelOctree(Eigen::Vector3d(0.5, 0.5, 0.5), 0.25, 0);
}

Point MakePoint(const Eigen::Vector3d& position) {
  Point point;
  point.position = position;
  return point;
}

// Nine coplanar points on z = 0.5, inside the root node.
void InsertPlanarPoints(VoxelOctree& node, const VoxelMapParams& params) {
  for (double x : {0.2, 0.4, 0.6}) {
    for (double y : {0.2, 0.4, 0.6}) {
      node.InsertPoint(MakePoint({x, y, 0.5}), params);
    }
  }
}

// The eight corners of a cube, which no plane fits.
void InsertScatteredPoints(VoxelOctree& node, const VoxelMapParams& params) {
  for (double x : {0.2, 0.8}) {
    for (double y : {0.2, 0.8}) {
      for (double z : {0.2, 0.8}) {
        node.InsertPoint(MakePoint({x, y, z}), params);
      }
    }
  }
}

TEST(VoxelOctree, StartsWithoutAPlane) {
  const VoxelOctree node = MakeRoot();

  EXPECT_FALSE(node.plane().has_value());
  EXPECT_EQ(node.PointCount(), 0U);
}

TEST(VoxelOctree, StoresInsertedPoints) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);

  EXPECT_EQ(node.PointCount(), 9U);
}

TEST(VoxelOctree, NeverStoresMoreThanTheCap) {
  VoxelMapParams params = MakeParams();
  params.max_points_per_voxel = 10;
  VoxelOctree node = MakeRoot();

  for (int i = 0; i < 100; ++i) {
    node.InsertPoint(MakePoint({0.5, 0.5, 0.5}), params);
  }

  EXPECT_EQ(node.PointCount(), 10U);
}

TEST(VoxelOctree, DoesNotFitAPlaneBeforeUpdate) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);

  EXPECT_FALSE(node.plane().has_value());
}

TEST(VoxelOctree, FitsAPlaneToCoplanarPoints) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);
  node.Update(params);

  ASSERT_TRUE(node.plane().has_value());
  const Plane& plane = *node.plane();
  EXPECT_NEAR(plane.centroid.x(), 0.4, 1e-12);
  EXPECT_NEAR(plane.centroid.y(), 0.4, 1e-12);
  EXPECT_NEAR(plane.centroid.z(), 0.5, 1e-12);
  // The eigenvector sign is arbitrary, so only the axis is meaningful.
  EXPECT_NEAR(std::abs(plane.normal.z()), 1.0, 1e-9);
  EXPECT_NEAR(plane.variance, 0.0, 1e-12);
}

TEST(VoxelOctree, NeedsMoreThanLayerInitPointsToFit) {
  VoxelMapParams params = MakeParams();
  params.layer_init_points = {9, 9, 9};
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);
  node.Update(params);

  EXPECT_FALSE(node.plane().has_value());
}

TEST(VoxelOctree, DoesNotFitAPlaneToScatteredPoints) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertScatteredPoints(node, params);
  node.Update(params);

  EXPECT_FALSE(node.plane().has_value());
}

TEST(VoxelOctree, SubdivisionMovesPointsOutOfTheParent) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertScatteredPoints(node, params);
  ASSERT_EQ(node.PointCount(), 8U);
  node.Update(params);

  EXPECT_EQ(node.PointCount(), 0U);
}

TEST(VoxelOctree, PointsDescendPastASubdividedNode) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertScatteredPoints(node, params);
  node.Update(params);

  node.InsertPoint(MakePoint({0.3, 0.3, 0.3}), params);

  EXPECT_EQ(node.PointCount(), 0U);
}

TEST(VoxelOctree, RefitKeepsThePlaneWhenPointsStayCoplanar) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);
  node.Update(params);
  ASSERT_TRUE(node.plane().has_value());

  node.InsertPoint(MakePoint({0.8, 0.8, 0.5}), params);
  node.Update(params);

  ASSERT_TRUE(node.plane().has_value());
  EXPECT_NEAR(std::abs(node.plane()->normal.z()), 1.0, 1e-9);
}

TEST(VoxelOctree, UpdateWithoutNewPointsLeavesThePlaneUnchanged) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);
  node.Update(params);
  ASSERT_TRUE(node.plane().has_value());
  const Eigen::Vector3d centroid = node.plane()->centroid;

  node.Update(params);

  ASSERT_TRUE(node.plane().has_value());
  EXPECT_EQ(node.plane()->centroid, centroid);
}

// Six points spread along the axes, which no plane fits. They land in four of
// the eight octants, leaving the other four without children.
void InsertCrossPoints(VoxelOctree& node, const VoxelMapParams& params) {
  node.InsertPoint(MakePoint({0.1, 0.5, 0.5}), params);
  node.InsertPoint(MakePoint({0.9, 0.5, 0.5}), params);
  node.InsertPoint(MakePoint({0.5, 0.1, 0.5}), params);
  node.InsertPoint(MakePoint({0.5, 0.9, 0.5}), params);
  node.InsertPoint(MakePoint({0.5, 0.5, 0.1}), params);
  node.InsertPoint(MakePoint({0.5, 0.5, 0.9}), params);
}

TEST(FindCorrespondence, ReturnsNullWhileStillAccumulating) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);

  EXPECT_EQ(node.FindCorrespondence({0.4, 0.4, 0.5}), nullptr);
}

TEST(FindCorrespondence, ReturnsTheNodeHoldingThePlane) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertPlanarPoints(node, params);
  node.Update(params);

  EXPECT_EQ(node.FindCorrespondence({0.4, 0.4, 0.5}), &node);
}

TEST(FindCorrespondence, ReturnsNullWhenTheSubtreeHasNoPlane) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertScatteredPoints(node, params);
  node.Update(params);

  EXPECT_EQ(node.FindCorrespondence({0.2, 0.2, 0.2}), nullptr);
}

TEST(FindCorrespondence, ReturnsNullWhenTheChildWasNeverCreated) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertCrossPoints(node, params);
  node.Update(params);

  // Nothing was ever inserted into the (+x, +y, +z) octant.
  EXPECT_EQ(node.FindCorrespondence({0.9, 0.9, 0.9}), nullptr);
}

TEST(FindCorrespondence, DescendsToAChildPlane) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();

  InsertCrossPoints(node, params);
  node.Update(params);
  ASSERT_EQ(node.PointCount(), 0U);

  // Fill the (-x, -y, -z) octant with points coplanar at z = 0.2.
  for (double x : {0.05, 0.15, 0.25, 0.35}) {
    for (double y : {0.05, 0.15, 0.25}) {
      node.InsertPoint(MakePoint({x, y, 0.2}), params);
    }
  }
  node.Update(params);

  const VoxelOctree* found = node.FindCorrespondence({0.15, 0.15, 0.2});
  ASSERT_NE(found, nullptr);
  EXPECT_NE(found, &node);
  ASSERT_TRUE(found->plane().has_value());
  EXPECT_NEAR(std::abs(found->plane()->normal.z()), 1.0, 1e-9);
}

}  // namespace
}  // namespace livo_slam
