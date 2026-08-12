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

Cloud MakeCloud(const std::vector<Eigen::Vector3d>& positions) {
  Cloud cloud;
  for (const Eigen::Vector3d& position : positions) {
    Point point;
    point.position = position;
    cloud.points.push_back(point);
  }
  return cloud;
}

TEST(VoxelMap, StartsEmpty) {
  const VoxelMap map(MakeParams());

  EXPECT_TRUE(map.Empty());
  EXPECT_EQ(map.VoxelCount(), 0U);
}

TEST(VoxelMap, EmptyCloudLeavesTheMapEmpty) {
  VoxelMap map(MakeParams());

  map.Insert(Cloud{});

  EXPECT_TRUE(map.Empty());
}

TEST(VoxelMap, PointsInOneVoxelCreateOneVoxel) {
  VoxelMap map(MakeParams());

  map.Insert(MakeCloud({{0.1, 0.1, 0.1}, {0.5, 0.5, 0.5}, {0.9, 0.9, 0.9}}));

  EXPECT_EQ(map.VoxelCount(), 1U);
}

TEST(VoxelMap, PointsInDifferentVoxelsCreateSeparateVoxels) {
  VoxelMap map(MakeParams());

  map.Insert(MakeCloud({{0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}, {0.5, 1.5, 0.5}}));

  EXPECT_EQ(map.VoxelCount(), 3U);
}

TEST(VoxelMap, NegativeCoordinatesGetTheirOwnVoxels) {
  VoxelMap map(MakeParams());

  map.Insert(MakeCloud({{-0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}));

  EXPECT_EQ(map.VoxelCount(), 2U);
}

TEST(VoxelMap, ReinsertingTheSameRegionReusesVoxels) {
  VoxelMap map(MakeParams());
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}});

  map.Insert(cloud);
  map.Insert(cloud);

  EXPECT_EQ(map.VoxelCount(), 2U);
}

TEST(VoxelMap, VoxelSizeDeterminesHowPointsAreGrouped) {
  const Cloud cloud = MakeCloud({{0.25, 0.25, 0.25}, {0.75, 0.75, 0.75}});

  VoxelMapParams coarse = MakeParams();
  coarse.voxel_size = 1.0;
  VoxelMap coarse_map(coarse);
  coarse_map.Insert(cloud);

  VoxelMapParams fine = MakeParams();
  fine.voxel_size = 0.5;
  VoxelMap fine_map(fine);
  fine_map.Insert(cloud);

  EXPECT_EQ(coarse_map.VoxelCount(), 1U);
  EXPECT_EQ(fine_map.VoxelCount(), 2U);
}

// Sixteen coplanar points at z = 0.5, all inside the voxel at index (0, 0, 0).
Cloud MakePlanarCloud() {
  Cloud cloud;
  for (double x : {0.2, 0.4, 0.6, 0.8}) {
    for (double y : {0.2, 0.4, 0.6, 0.8}) {
      Point point;
      point.position = {x, y, 0.5};
      cloud.points.push_back(point);
    }
  }
  return cloud;
}

TEST(FindPlane, EmptyMapMatchesNothing) {
  const VoxelMap map(MakeParams());

  EXPECT_FALSE(map.FindPlane({0.5, 0.5, 0.5}).has_value());
}

TEST(FindPlane, MatchesAPlaneInTheContainingVoxel) {
  VoxelMap map(MakeParams());
  map.Insert(MakePlanarCloud());

  const auto plane = map.FindPlane({0.5, 0.5, 0.55});

  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(plane->centroid.z(), 0.5, 1e-12);
  EXPECT_NEAR(std::abs(plane->normal.z()), 1.0, 1e-9);
}

TEST(FindPlane, RejectsPointsBeyondTheMatchDistance) {
  VoxelMapParams params = MakeParams();
  params.max_match_distance = 0.1;
  VoxelMap map(params);
  map.Insert(MakePlanarCloud());

  EXPECT_TRUE(map.FindPlane({0.5, 0.5, 0.55}).has_value());
  EXPECT_FALSE(map.FindPlane({0.5, 0.5, 0.9}).has_value());
}

TEST(FindPlane, FallsBackToTheNeighborVoxel) {
  VoxelMap map(MakeParams());
  map.Insert(MakePlanarCloud());
  ASSERT_EQ(map.VoxelCount(), 1U);

  // Just inside the empty voxel at (1, 0, 0), close to the shared face.
  const auto plane = map.FindPlane({1.1, 0.5, 0.55});

  ASSERT_TRUE(plane.has_value());
  EXPECT_NEAR(std::abs(plane->normal.z()), 1.0, 1e-9);
}

TEST(FindPlane, DoesNotLeaveAVoxelFromItsMiddle) {
  VoxelMap map(MakeParams());
  map.Insert(MakePlanarCloud());

  // Centre of the empty voxel at (1, 0, 0): no axis is past a quarter, so no
  // neighbor is tried even though the plane next door is within range.
  EXPECT_FALSE(map.FindPlane({1.5, 0.5, 0.5}).has_value());
}

TEST(FindPlane, MatchesNothingFarFromTheMap) {
  VoxelMap map(MakeParams());
  map.Insert(MakePlanarCloud());

  EXPECT_FALSE(map.FindPlane({9.0, 9.0, 9.0}).has_value());
}

TEST(FindPlane, MatchesNothingWhileVoxelsAreStillAccumulating) {
  VoxelMap map(MakeParams());
  Cloud cloud;
  for (double x : {0.2, 0.4, 0.6}) {
    Point point;
    point.position = {x, 0.2, 0.5};
    cloud.points.push_back(point);
  }
  map.Insert(cloud);

  ASSERT_EQ(map.VoxelCount(), 1U);
  EXPECT_FALSE(map.FindPlane({0.4, 0.2, 0.5}).has_value());
}

}  // namespace
}  // namespace livo_slam
