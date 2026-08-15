#include "core/estimator/lidar/lidar_update.h"

#include "core/estimator/lidar/lidar_residual.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

VoxelMapParams MakeMapParams() {
  VoxelMapParams params;
  params.voxel_size = 1.0;
  params.layer_init_points = {5, 5, 5};
  params.max_points_per_voxel = 50;
  params.max_plane_variance = 0.01;
  params.max_match_distance = 0.5;
  return params;
}

LidarUpdateParams MakeUpdateParams() {
  LidarUpdateParams params;
  params.max_iterations = 5;
  params.point_noise_std = 0.03;
  params.rotation_epsilon = 0.001;
  params.position_epsilon = 0.001;
  return params;
}

// A map holding one plane at z = 0.5, centred on (0.5, 0.5, 0.5).
VoxelMap MakeMapWithPlane() {
  VoxelMap map(MakeMapParams());

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

// Points that lie exactly on that plane when the state is the identity.
Cloud MakeAlignedCloud() {
  Cloud cloud;
  for (double x : {0.3, 0.5, 0.7}) {
    for (double y : {0.3, 0.5, 0.7}) {
      Point point;
      point.position = {x, y, 0.5};
      cloud.points.push_back(point);
    }
  }
  return cloud;
}

Estimate MakePrior(const Eigen::Vector3d& position) {
  Estimate prior;
  prior.state.position = position;
  prior.covariance = ErrorMatrix::Identity() * 0.01;
  prior.timestamp = 17.0;
  return prior;
}

// A patch of one plane does not fix the pose uniquely - a tilt can absorb part
// of a translation - so the residuals are what the update actually minimises.
double WorstResidual(const State& state, const Cloud& cloud,
                     const VoxelMap& map) {
  double worst = 0.0;
  for (const PointToPlaneMatch& match : BuildLidarMatches(cloud, state, map)) {
    worst = std::max(worst, std::abs(match.distance));
  }
  return worst;
}

TEST(UpdateWithLidar, PullsThePointsOntoThePlane) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud = MakeAlignedCloud();
  const Estimate prior = MakePrior({0.0, 0.0, 0.1});

  const Estimate posterior =
      UpdateWithLidar(prior, cloud, map, MakeUpdateParams());

  EXPECT_NEAR(WorstResidual(prior.state, cloud, map), 0.1, 1e-9);
  EXPECT_LT(WorstResidual(posterior.state, cloud, map), 0.01);
}

TEST(UpdateWithLidar, CorrectsFromEitherSideOfThePlane) {
  const VoxelMap map = MakeMapWithPlane();
  const Cloud cloud = MakeAlignedCloud();
  const LidarUpdateParams params = MakeUpdateParams();

  const Estimate above =
      UpdateWithLidar(MakePrior({0.0, 0.0, 0.1}), cloud, map, params);
  const Estimate below =
      UpdateWithLidar(MakePrior({0.0, 0.0, -0.1}), cloud, map, params);

  EXPECT_LT(WorstResidual(above.state, cloud, map), 0.01);
  EXPECT_LT(WorstResidual(below.state, cloud, map), 0.01);
  EXPECT_GT(above.state.position.z(), 0.0);
  EXPECT_LT(below.state.position.z(), 0.0);
}

TEST(UpdateWithLidar, LeavesAnAlignedPoseAlone) {
  const VoxelMap map = MakeMapWithPlane();
  const Estimate prior = MakePrior({0.0, 0.0, 0.0});

  const Estimate posterior = UpdateWithLidar(prior, MakeAlignedCloud(), map,
                                             MakeUpdateParams());

  EXPECT_LT((posterior.state.position - prior.state.position).norm(), 1e-9);
}

TEST(UpdateWithLidar, ShrinksTheCovariance) {
  const VoxelMap map = MakeMapWithPlane();
  const Estimate prior = MakePrior({0.0, 0.0, 0.1});

  const Estimate posterior = UpdateWithLidar(prior, MakeAlignedCloud(), map,
                                             MakeUpdateParams());

  EXPECT_LT(posterior.covariance(kIdxPosition + 2, kIdxPosition + 2),
            prior.covariance(kIdxPosition + 2, kIdxPosition + 2));
}

TEST(UpdateWithLidar, KeepsTheTimestamp) {
  const VoxelMap map = MakeMapWithPlane();
  const Estimate prior = MakePrior({0.0, 0.0, 0.1});

  const Estimate posterior = UpdateWithLidar(prior, MakeAlignedCloud(), map,
                                             MakeUpdateParams());

  EXPECT_DOUBLE_EQ(posterior.timestamp, prior.timestamp);
}

TEST(UpdateWithLidar, ReturnsThePriorWhenTheMapHasNoPlanes) {
  const VoxelMap empty_map(MakeMapParams());
  const Estimate prior = MakePrior({0.0, 0.0, 0.1});

  const Estimate posterior = UpdateWithLidar(prior, MakeAlignedCloud(),
                                             empty_map, MakeUpdateParams());

  EXPECT_EQ(posterior.state.position, prior.state.position);
  EXPECT_TRUE(posterior.covariance.isApprox(prior.covariance));
}

TEST(UpdateWithLidar, ReturnsThePriorWhenTooFewPointsMatch) {
  const VoxelMap map = MakeMapWithPlane();
  const Estimate prior = MakePrior({0.0, 0.0, 0.1});

  // Five points is below kMinMatches, which is six.
  Cloud cloud;
  for (double x : {0.3, 0.4, 0.5, 0.6, 0.7}) {
    Point point;
    point.position = {x, 0.5, 0.5};
    cloud.points.push_back(point);
  }

  const Estimate posterior =
      UpdateWithLidar(prior, cloud, map, MakeUpdateParams());

  EXPECT_EQ(posterior.state.position, prior.state.position);
}

TEST(UpdateWithLidar, ASingleIterationAlreadyImproves) {
  const VoxelMap map = MakeMapWithPlane();
  const Estimate prior = MakePrior({0.0, 0.0, 0.1});

  LidarUpdateParams params = MakeUpdateParams();
  params.max_iterations = 1;

  const Estimate posterior =
      UpdateWithLidar(prior, MakeAlignedCloud(), map, params);

  EXPECT_LT(std::abs(posterior.state.position.z()),
            std::abs(prior.state.position.z()));
}

}  // namespace
}  // namespace livo_slam
