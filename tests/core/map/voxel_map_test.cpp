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

}  // namespace
}  // namespace livo_slam
