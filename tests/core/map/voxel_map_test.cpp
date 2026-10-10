#include "core/map/voxel_map.h"

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

std::vector<Point> MakePoints(const std::vector<Eigen::Vector3d>& positions) {
  std::vector<Point> points;
  for (const Eigen::Vector3d& position : positions) {
    Point point;
    point.position = position;
    points.push_back(point);
  }
  return points;
}

TEST(VoxelMap, StartsEmpty) {
  const VoxelMap map(MakeParams());

  EXPECT_TRUE(map.Empty());
  EXPECT_EQ(map.VoxelCount(), 0U);
}

TEST(VoxelMap, EmptyCloudLeavesTheMapEmpty) {
  VoxelMap map(MakeParams());

  map.InsertPoints({});

  EXPECT_TRUE(map.Empty());
}

TEST(VoxelMap, PointsInOneVoxelCreateOneVoxel) {
  VoxelMap map(MakeParams());

  map.InsertPoints(
      MakePoints({{0.1, 0.1, 0.1}, {0.5, 0.5, 0.5}, {0.9, 0.9, 0.9}}));

  EXPECT_EQ(map.VoxelCount(), 1U);
}

TEST(VoxelMap, PointsInDifferentVoxelsCreateSeparateVoxels) {
  VoxelMap map(MakeParams());

  map.InsertPoints(
      MakePoints({{0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}, {0.5, 1.5, 0.5}}));

  EXPECT_EQ(map.VoxelCount(), 3U);
}

TEST(VoxelMap, NegativeCoordinatesGetTheirOwnVoxels) {
  VoxelMap map(MakeParams());

  map.InsertPoints(MakePoints({{-0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}));

  EXPECT_EQ(map.VoxelCount(), 2U);
}

TEST(VoxelMap, ReinsertingTheSameRegionReusesVoxels) {
  VoxelMap map(MakeParams());
  const std::vector<Point> cloud =
      MakePoints({{0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}});

  map.InsertPoints(cloud);
  map.InsertPoints(cloud);

  EXPECT_EQ(map.VoxelCount(), 2U);
}

TEST(VoxelMap, VoxelSizeDeterminesHowPointsAreGrouped) {
  const std::vector<Point> cloud =
      MakePoints({{0.25, 0.25, 0.25}, {0.75, 0.75, 0.75}});

  VoxelMapParams coarse = MakeParams();
  coarse.voxel_size = 1.0;
  VoxelMap coarse_map(coarse);
  coarse_map.InsertPoints(cloud);

  VoxelMapParams fine = MakeParams();
  fine.voxel_size = 0.5;
  VoxelMap fine_map(fine);
  fine_map.InsertPoints(cloud);

  EXPECT_EQ(coarse_map.VoxelCount(), 1U);
  EXPECT_EQ(fine_map.VoxelCount(), 2U);
}

// Sixteen coplanar points at z = 0.5, all inside the voxel at index (0, 0, 0).
std::vector<Point> MakePlanarPoints() {
  std::vector<Point> points;
  for (double x : {0.2, 0.4, 0.6, 0.8}) {
    for (double y : {0.2, 0.4, 0.6, 0.8}) {
      Point point;
      point.position = {x, y, 0.5};
      points.push_back(point);
    }
  }
  return points;
}

TEST(FindPlane, EmptyMapMatchesNothing) {
  const VoxelMap map(MakeParams());

  EXPECT_FALSE(map.FindPlane({0.5, 0.5, 0.5}).has_value());
}

TEST(FindPlane, MatchesAPlaneInTheContainingVoxel) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  const auto plane = map.FindPlane({0.5, 0.5, 0.55});

  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(plane->centroid.z(), 0.5, 1e-12);
  EXPECT_NEAR(std::abs(plane->normal.z()), 1.0, 1e-9);
}

TEST(FindPlane, RejectsPointsBeyondTheMatchDistance) {
  VoxelMapParams params = MakeParams();
  params.max_match_distance = 0.1;
  VoxelMap map(params);
  map.InsertPoints(MakePlanarPoints());

  EXPECT_TRUE(map.FindPlane({0.5, 0.5, 0.55}).has_value());
  EXPECT_FALSE(map.FindPlane({0.5, 0.5, 0.9}).has_value());
}

TEST(FindPlane, FallsBackToTheNeighborVoxel) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());
  ASSERT_EQ(map.VoxelCount(), 1U);

  // Just inside the empty voxel at (1, 0, 0), close to the shared face.
  const auto plane = map.FindPlane({1.1, 0.5, 0.55});

  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(std::abs(plane->normal.z()), 1.0, 1e-9);
}

TEST(FindPlane, DoesNotLeaveAVoxelFromItsMiddle) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  // Center of the empty voxel at (1, 0, 0): no axis is past a quarter, so no
  // neighbor is tried even though the plane next door is within range.
  EXPECT_FALSE(map.FindPlane({1.5, 0.5, 0.5}).has_value());
}

TEST(FindPlane, MatchesNothingFarFromTheMap) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  EXPECT_FALSE(map.FindPlane({9.0, 9.0, 9.0}).has_value());
}

TEST(FindPlane, MatchesNothingWhileVoxelsAreStillAccumulating) {
  VoxelMap map(MakeParams());
  std::vector<Point> cloud;
  for (double x : {0.2, 0.4, 0.6}) {
    Point point;
    point.position = {x, 0.2, 0.5};
    cloud.push_back(point);
  }
  map.InsertPoints(cloud);

  ASSERT_EQ(map.VoxelCount(), 1U);
  EXPECT_FALSE(map.FindPlane({0.4, 0.2, 0.5}).has_value());
}

TEST(CollectVisualPoints, EmptyMapCollectsNothing) {
  const VoxelMap map(MakeParams());

  EXPECT_TRUE(map.CollectVisualPoints(MakePlanarPoints()).empty());
}

TEST(CollectVisualPoints, EmptyCloudCollectsNothing) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  EXPECT_TRUE(map.CollectVisualPoints({}).empty());
}

TEST(CollectVisualPoints, VoxelsWithoutVisualPointsCollectNothing) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());
  ASSERT_EQ(map.VoxelCount(), 1U);

  EXPECT_TRUE(map.CollectVisualPoints(MakePlanarPoints()).empty());
}

TEST(CollectVisualPoints, CloudAwayFromTheMapCollectsNothing) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  EXPECT_TRUE(map.CollectVisualPoints(MakePoints({{9.0, 9.0, 9.0}})).empty());
}

VisualPoint MakeVisualPoint(const Eigen::Vector3d& position) {
  VisualPoint visual_point;
  visual_point.position = position;
  return visual_point;
}

VisualPatch MakeVisualPatch(double inv_exposure) {
  VisualPatch patch;
  patch.inv_exposure = inv_exposure;
  return patch;
}

TEST(InsertVisualPoints, StoresAPointInItsVoxel) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  map.InsertVisualPoints({MakeVisualPoint({0.5, 0.5, 0.5})}, {}, 0);

  const auto collected = map.CollectVisualPoints(MakePlanarPoints());
  ASSERT_EQ(collected.size(), 1U);
  EXPECT_EQ(collected[0]->position, Eigen::Vector3d(0.5, 0.5, 0.5));
}

TEST(InsertVisualPoints, SkipsPositionsWithNoVoxel) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());
  ASSERT_EQ(map.VoxelCount(), 1U);

  // Nothing was ever inserted at (9, 9, 9), so there is no voxel to hold it.
  map.InsertVisualPoints({MakeVisualPoint({9.0, 9.0, 9.0})}, {}, 0);

  EXPECT_EQ(map.VoxelCount(), 1U);
  EXPECT_TRUE(map.CollectVisualPoints(MakePoints({{9.0, 9.0, 9.0}})).empty());
}

TEST(InsertVisualPoints, PointsGoToTheVoxelHoldingTheirPosition) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePoints({{0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}}));

  map.InsertVisualPoints(
      {MakeVisualPoint({0.5, 0.5, 0.5}), MakeVisualPoint({1.5, 0.5, 0.5})}, {},
      0);

  EXPECT_EQ(map.CollectVisualPoints(MakePoints({{0.5, 0.5, 0.5}})).size(), 1U);
  EXPECT_EQ(map.CollectVisualPoints(MakePoints({{1.5, 0.5, 0.5}})).size(), 1U);
}

TEST(InsertVisualPoints, AppendsAPatchToAnExistingPoint) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());
  map.InsertVisualPoints({MakeVisualPoint({0.5, 0.5, 0.5})}, {}, 0);

  map.InsertVisualPoints({}, {{{0.5, 0.5, 0.5}, MakeVisualPatch(0.7)}}, 0);

  const auto collected = map.CollectVisualPoints(MakePlanarPoints());
  ASSERT_EQ(collected.size(), 1U);
  ASSERT_EQ(collected[0]->patches.size(), 1U);
  EXPECT_DOUBLE_EQ(collected[0]->patches[0].inv_exposure, 0.7);
}

TEST(InsertVisualPoints, AddsPointsAndPatchesInOneCall) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());
  map.InsertVisualPoints({MakeVisualPoint({0.2, 0.2, 0.5})}, {}, 0);

  map.InsertVisualPoints({MakeVisualPoint({0.8, 0.8, 0.5})},
                         {{{0.2, 0.2, 0.5}, MakeVisualPatch(0.7)}}, 0);

  const auto collected = map.CollectVisualPoints(MakePlanarPoints());
  ASSERT_EQ(collected.size(), 2U);

  std::size_t with_patch = 0;
  for (const VisualPoint* point : collected) {
    with_patch += point->patches.size();
  }
  EXPECT_EQ(with_patch, 1U);
}

TEST(InsertVisualPoints, IgnoresAPatchWithNoMatchingPoint) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());
  map.InsertVisualPoints({MakeVisualPoint({0.5, 0.5, 0.5})}, {}, 0);

  map.InsertVisualPoints({}, {{{0.2, 0.2, 0.5}, MakeVisualPatch(0.7)}}, 0);

  const auto collected = map.CollectVisualPoints(MakePlanarPoints());
  ASSERT_EQ(collected.size(), 1U);
  EXPECT_TRUE(collected[0]->patches.empty());
}

TEST(InsertVisualPoints, EmptyInputChangesNothing) {
  VoxelMap map(MakeParams());
  map.InsertPoints(MakePlanarPoints());

  map.InsertVisualPoints({}, {}, 0);

  EXPECT_TRUE(map.CollectVisualPoints(MakePlanarPoints()).empty());
}

}  // namespace
}  // namespace livo_slam
