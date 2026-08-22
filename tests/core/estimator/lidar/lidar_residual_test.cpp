#include "core/estimator/lidar/lidar_residual.h"

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

// A map holding one plane at z = 0.5, centerd on (0.5, 0.5, 0.5).
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

PointToPlaneMatch MakeMatch() {
  PointToPlaneMatch match;
  match.point_imu = {0.7, 1.3, -0.4};
  match.normal = Eigen::Vector3d(0.2, -0.5, 0.84).normalized();
  match.distance = 0.05;
  return match;
}

State MakeState() {
  State state;
  state.rotation = Sophus::SO3d::exp({0.3, -0.2, 0.7});
  state.position = {1.0, -2.0, 0.5};
  return state;
}

TEST(BuildLidarSystem, NoMatchesGivesAnEmptySystem) {
  const IekfSystem system = BuildLidarSystem({}, MakeState(), 0.03);

  EXPECT_TRUE(system.hth.isZero());
  EXPECT_TRUE(system.htz.isZero());
}

TEST(BuildLidarSystem, TouchesOnlyTheRotationAndPositionBlock) {
  const IekfSystem system = BuildLidarSystem({MakeMatch()}, MakeState(), 0.03);

  const Eigen::Matrix<double, 6, 6> pose_block =
      system.hth.block<6, 6>(kIdxRotation, kIdxRotation);

  EXPECT_FALSE(pose_block.isZero());
  EXPECT_TRUE(system.hth.bottomRows(kErrorDim - 6).isZero());
  EXPECT_TRUE(system.hth.rightCols(kErrorDim - 6).isZero());
  EXPECT_TRUE(system.htz.tail(kErrorDim - 6).isZero());
}

TEST(BuildLidarSystem, JacobianMatchesFiniteDifferences) {
  const State state = MakeState();
  const PointToPlaneMatch match = MakeMatch();
  const double point_noise_std = 0.03;
  const double weight = 1.0 / (point_noise_std * point_noise_std);

  const IekfSystem system = BuildLidarSystem({match}, state, point_noise_std);

  // htz = weight * jacobian * (-distance), so the jacobian divides back out.
  const Eigen::Matrix<double, 6, 1> jacobian =
      system.htz.segment<6>(kIdxRotation) / (weight * -match.distance);

  // The centroid is constant, so it drops out of the differences.
  const auto distance = [&](const ErrorState& delta) {
    const State perturbed = BoxPlus(state, delta);
    return match.normal.dot(perturbed.rotation * match.point_imu +
                            perturbed.position);
  };

  const double eps = 1e-6;
  const double at_zero = distance(ErrorState::Zero());
  for (int i = 0; i < 6; ++i) {
    ErrorState delta = ErrorState::Zero();
    delta(kIdxRotation + i) = eps;
    EXPECT_NEAR(jacobian(i), (distance(delta) - at_zero) / eps, 1e-5)
        << "component " << i;
  }
}

TEST(BuildLidarSystem, WeightsByInverseNoiseVariance) {
  const State state = MakeState();
  const PointToPlaneMatch match = MakeMatch();

  const IekfSystem coarse = BuildLidarSystem({match}, state, 0.06);
  const IekfSystem fine = BuildLidarSystem({match}, state, 0.03);

  EXPECT_TRUE(fine.hth.isApprox(4.0 * coarse.hth));
  EXPECT_TRUE(fine.htz.isApprox(4.0 * coarse.htz));
}

TEST(BuildLidarSystem, AccumulatesOverMatches) {
  const State state = MakeState();
  const PointToPlaneMatch match = MakeMatch();

  const IekfSystem one = BuildLidarSystem({match}, state, 0.03);
  const IekfSystem two = BuildLidarSystem({match, match}, state, 0.03);

  EXPECT_TRUE(two.hth.isApprox(2.0 * one.hth));
  EXPECT_TRUE(two.htz.isApprox(2.0 * one.htz));
}

TEST(BuildLidarSystem, ResidualOpposesTheSignedDistance) {
  const State state = MakeState();
  PointToPlaneMatch positive = MakeMatch();
  PointToPlaneMatch negative = positive;
  negative.distance = -positive.distance;

  const IekfSystem a = BuildLidarSystem({positive}, state, 0.03);
  const IekfSystem b = BuildLidarSystem({negative}, state, 0.03);

  EXPECT_TRUE(a.htz.isApprox(-b.htz));
  EXPECT_TRUE(a.hth.isApprox(b.hth));
}

}  // namespace
}  // namespace livo_slam
