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

VisualPoint MakeVisualPoint(const Eigen::Vector3d& position) {
  VisualPoint visual_point;
  visual_point.position = position;
  return visual_point;
}

// Marked by its inverse exposure so tests can tell patches apart.
VisualPatch MakeVisualPatch(double inv_exposure) {
  VisualPatch patch;
  patch.inv_exposure = inv_exposure;
  return patch;
}

// A patch looking at the origin from `camera`, so the viewing angle against a
// point's normal is what decides which one gets dropped.
VisualPatch MakeVisualPatchFrom(double inv_exposure,
                                const Eigen::Vector3d& camera) {
  VisualPatch patch;
  patch.inv_exposure = inv_exposure;
  patch.T_world_camera = Sophus::SE3d(Eigen::Quaterniond::Identity(), camera);
  return patch;
}

std::vector<const VisualPoint*> Collect(const VoxelOctree& node) {
  std::vector<const VisualPoint*> visual_points;
  node.CollectVisualPoints(visual_points);
  return visual_points;
}

TEST(InsertVisualPoint, StoresThePoint) {
  VoxelOctree node = MakeRoot();

  node.InsertVisualPoint(MakeVisualPoint({0.5, 0.5, 0.5}));

  const auto collected = Collect(node);
  ASSERT_EQ(collected.size(), 1U);
  EXPECT_EQ(collected[0]->position, Eigen::Vector3d(0.5, 0.5, 0.5));
}

TEST(InsertVisualPoint, KeepsSeveralPoints) {
  VoxelOctree node = MakeRoot();

  node.InsertVisualPoint(MakeVisualPoint({0.2, 0.2, 0.2}));
  node.InsertVisualPoint(MakeVisualPoint({0.8, 0.8, 0.8}));

  EXPECT_EQ(Collect(node).size(), 2U);
}

TEST(InsertVisualPoint, StaysAtThisNodeWhenSubdivided) {
  const VoxelMapParams params = MakeParams();
  VoxelOctree node = MakeRoot();
  InsertScatteredPoints(node, params);
  node.Update(params);
  ASSERT_EQ(node.PointCount(), 0U);

  // Visual points are flat: subdivision does not push them into children.
  node.InsertVisualPoint(MakeVisualPoint({0.2, 0.2, 0.2}));

  EXPECT_EQ(Collect(node).size(), 1U);
}

TEST(AddVisualPatch, AppendsToTheMatchingPoint) {
  VoxelOctree node = MakeRoot();
  node.InsertVisualPoint(MakeVisualPoint({0.2, 0.2, 0.2}));
  node.InsertVisualPoint(MakeVisualPoint({0.8, 0.8, 0.8}));

  node.AddVisualPatch({0.8, 0.8, 0.8}, MakeVisualPatch(0.7), 0);

  const auto collected = Collect(node);
  ASSERT_EQ(collected.size(), 2U);
  EXPECT_TRUE(collected[0]->patches.empty());
  ASSERT_EQ(collected[1]->patches.size(), 1U);
  EXPECT_DOUBLE_EQ(collected[1]->patches[0].inv_exposure, 0.7);
}

TEST(AddVisualPatch, AccumulatesPatchesOnOnePoint) {
  VoxelOctree node = MakeRoot();
  node.InsertVisualPoint(MakeVisualPoint({0.5, 0.5, 0.5}));

  node.AddVisualPatch({0.5, 0.5, 0.5}, MakeVisualPatch(0.7), 0);
  node.AddVisualPatch({0.5, 0.5, 0.5}, MakeVisualPatch(0.6), 0);

  const auto collected = Collect(node);
  ASSERT_EQ(collected[0]->patches.size(), 2U);
  EXPECT_DOUBLE_EQ(collected[0]->patches[0].inv_exposure, 0.7);
  EXPECT_DOUBLE_EQ(collected[0]->patches[1].inv_exposure, 0.6);
}

TEST(AddVisualPatch, KeepsAtMostTheGivenNumberOfPatches) {
  VoxelOctree node = MakeRoot();
  VisualPoint point = MakeVisualPoint({0.5, 0.5, 0.5});
  point.normal = {0.0, 0.0, 1.0};
  node.InsertVisualPoint(point);

  for (int i = 0; i < 5; ++i) {
    node.AddVisualPatch({0.5, 0.5, 0.5},
                        MakeVisualPatchFrom(0.1 * i, {0.5, 0.5, 1.5}), 3);
  }

  EXPECT_EQ(Collect(node)[0]->patches.size(), 3U);
}

TEST(AddVisualPatch, DropsTheMostObliquePatch) {
  VoxelOctree node = MakeRoot();
  VisualPoint point = MakeVisualPoint({0.5, 0.5, 0.5});
  point.normal = {0.0, 0.0, 1.0};
  node.InsertVisualPoint(point);

  // Straight on, then almost edge on, then straight on again.
  node.AddVisualPatch({0.5, 0.5, 0.5},
                      MakeVisualPatchFrom(0.7, {0.5, 0.5, 1.5}), 2);
  node.AddVisualPatch({0.5, 0.5, 0.5},
                      MakeVisualPatchFrom(0.6, {1.5, 0.5, 0.51}), 2);
  node.AddVisualPatch({0.5, 0.5, 0.5},
                      MakeVisualPatchFrom(0.5, {0.5, 0.5, 2.5}), 2);

  const auto collected = Collect(node);
  ASSERT_EQ(collected[0]->patches.size(), 2U);
  EXPECT_DOUBLE_EQ(collected[0]->patches[0].inv_exposure, 0.7);
  EXPECT_DOUBLE_EQ(collected[0]->patches[1].inv_exposure, 0.5);
}

TEST(AddVisualPatch, ZeroKeepsEveryPatch) {
  VoxelOctree node = MakeRoot();
  node.InsertVisualPoint(MakeVisualPoint({0.5, 0.5, 0.5}));

  for (int i = 0; i < 40; ++i) {
    node.AddVisualPatch({0.5, 0.5, 0.5}, MakeVisualPatch(0.1), 0);
  }

  EXPECT_EQ(Collect(node)[0]->patches.size(), 40U);
}

TEST(AddVisualPatch, DoesNothingWhenNoPointMatches) {
  VoxelOctree node = MakeRoot();
  node.InsertVisualPoint(MakeVisualPoint({0.2, 0.2, 0.2}));

  node.AddVisualPatch({0.9, 0.9, 0.9}, MakeVisualPatch(0.7), 0);

  const auto collected = Collect(node);
  ASSERT_EQ(collected.size(), 1U);
  EXPECT_TRUE(collected[0]->patches.empty());
}

TEST(AddVisualPatch, DoesNothingOnAnEmptyNode) {
  VoxelOctree node = MakeRoot();

  node.AddVisualPatch({0.5, 0.5, 0.5}, MakeVisualPatch(0.7), 0);

  EXPECT_TRUE(Collect(node).empty());
}

}  // namespace
}  // namespace livo_slam
