#include "core/estimator/lidar_residual.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

VoxelMapParams MakeParams() {
  VoxelMapParams params;
  params.voxel_size = 1.0;
  params.layer_init_points = {5, 5, 5};
  params.max_points_per_voxel = 50;
  params.max_plane_variance = 0.01;
  params.max_match_distance = 0.5;
  return params;
}

// A map holding one plane at z = 0.5, centred on (0.5, 0.5, 0.5).
VoxelMap MakeMapWithPlane() {
  VoxelMap map(MakeParams());

  Cloud cloud;
  for (double x : {0.2, 0.4, 0.6, 0.8}) {
    for (double y : {0.2, 0.4, 0.6, 0.8}) {
      Point point;
      point.position = {x, y, 0.5};
      cloud.points.push_back(point);
    }
  }
  map.Insert(cloud);
  return map;
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

TEST(BuildLidarMatches, EmptyCloudGivesNoMatches) {
  const VoxelMap map = MakeMapWithPlane();

  EXPECT_TRUE(BuildLidarMatches(Cloud{}, State{}, map).empty());
}

TEST(BuildLidarMatches, EmptyMapGivesNoMatches) {
  const VoxelMap map(MakeParams());
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.55}});

  EXPECT_TRUE(BuildLidarMatches(cloud, State{}, map).empty());
}

TEST(BuildLidarMatches, MatchesAPointAgainstThePlane) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.55}});

  const auto matches = BuildLidarMatches(cloud, State{}, map);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_EQ(matches[0].point_imu, Eigen::Vector3d(0.5, 0.5, 0.55));
  EXPECT_NEAR(std::abs(matches[0].normal.z()), 1.0, 1e-9);
  EXPECT_NEAR(std::abs(matches[0].distance), 0.05, 1e-9);
}

TEST(BuildLidarMatches, DistanceFlipsSignAcrossThePlane) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.55}, {0.5, 0.5, 0.45}});

  const auto matches = BuildLidarMatches(cloud, State{}, map);

  ASSERT_EQ(matches.size(), 2U);
  EXPECT_NEAR(matches[0].distance, -matches[1].distance, 1e-12);
  EXPECT_NEAR(std::abs(matches[0].distance), 0.05, 1e-9);
}

TEST(BuildLidarMatches, DistanceIsZeroOnThePlane) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.5}});

  const auto matches = BuildLidarMatches(cloud, State{}, map);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_NEAR(matches[0].distance, 0.0, 1e-12);
}

TEST(BuildLidarMatches, DropsPointsWithNoPlaneNearby) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud =
      MakeCloud({{0.5, 0.5, 0.55}, {9.0, 9.0, 9.0}, {0.4, 0.4, 0.45}});

  const auto matches = BuildLidarMatches(cloud, State{}, map);

  EXPECT_EQ(matches.size(), 2U);
}

TEST(BuildLidarMatches, AppliesTheStateTranslation) {
  const VoxelMap map = MakeMapWithPlane();
  // The point is on the plane once the state shifts it up by 0.05.
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.45}});

  State state;
  state.position = Eigen::Vector3d(0.0, 0.0, 0.05);

  const auto matches = BuildLidarMatches(cloud, state, map);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_NEAR(matches[0].distance, 0.0, 1e-12);
}

TEST(BuildLidarMatches, AppliesTheStateRotation) {
  const VoxelMap map = MakeMapWithPlane();
  // A half turn about z maps (-0.5, -0.5, 0.55) onto (0.5, 0.5, 0.55).
  const Cloud cloud = MakeCloud({{-0.5, -0.5, 0.55}});

  State state;
  state.rotation = Sophus::SO3d::rotZ(M_PI);

  const auto matches = BuildLidarMatches(cloud, state, map);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_NEAR(std::abs(matches[0].distance), 0.05, 1e-9);
}

TEST(BuildLidarMatches, KeepsThePointInTheBodyFrame) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud = MakeCloud({{0.5, 0.5, 0.45}});

  State state;
  state.position = Eigen::Vector3d(0.0, 0.0, 0.05);

  const auto matches = BuildLidarMatches(cloud, state, map);

  ASSERT_EQ(matches.size(), 1U);
  EXPECT_EQ(matches[0].point_imu, Eigen::Vector3d(0.5, 0.5, 0.45));
}

}  // namespace
}  // namespace livo_slam
