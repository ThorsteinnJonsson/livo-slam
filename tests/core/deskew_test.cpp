#include "core/lidar/deskew.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

Trajectory MakeTrajectory(double yaw_rate, const Eigen::Vector3d& velocity) {
  Trajectory trajectory;
  for (int i = 0; i <= 2; ++i) {
    const double t = 0.05 * static_cast<double>(i);
    ImuPose pose;
    pose.timestamp = t;
    pose.pose = Sophus::SE3d(Sophus::SO3d::rotZ(yaw_rate * t), velocity * t);
    trajectory.push_back(pose);
  }
  return trajectory;
}

State MakeReference(double yaw_rate, const Eigen::Vector3d& velocity) {
  State reference;
  reference.rotation = Sophus::SO3d::rotZ(yaw_rate * 0.1);
  reference.position = velocity * 0.1;
  return reference;
}

Cloud MakeCloud(const std::vector<double>& timestamps) {
  Cloud cloud;
  cloud.start_time = 0.0;
  cloud.end_time = 0.1;
  for (double t : timestamps) {
    Point point;
    point.timestamp = t;
    point.intensity = 7.0F;
    point.position = Eigen::Vector3d(1.0, 0.0, 0.0);
    cloud.points.push_back(point);
  }
  return cloud;
}

TEST(Deskew, EmptyTrajectoryReturnsCloudUnchanged) {
  const Cloud cloud = MakeCloud({0.0, 0.05});
  const Cloud out = Deskew(cloud, Trajectory{}, State{}, Sophus::SE3d{});

  ASSERT_EQ(out.points.size(), cloud.points.size());
  for (std::size_t i = 0; i < out.points.size(); ++i) {
    EXPECT_EQ(out.points[i].position, cloud.points[i].position);
  }
}

TEST(Deskew, StationaryPlatformLeavesPointsUnchanged) {
  const Trajectory trajectory = MakeTrajectory(0.0, Eigen::Vector3d::Zero());
  const State reference = MakeReference(0.0, Eigen::Vector3d::Zero());
  const Cloud cloud = MakeCloud({0.0, 0.03, 0.1});

  const Cloud out = Deskew(cloud, trajectory, reference, Sophus::SE3d{});

  for (std::size_t i = 0; i < out.points.size(); ++i) {
    EXPECT_LT((out.points[i].position - cloud.points[i].position).norm(),
              1e-12);
  }
}

TEST(Deskew, PointAtReferenceTimeIsUnchanged) {
  const Trajectory trajectory =
      MakeTrajectory(1.0, Eigen::Vector3d(2.0, 0.0, 0.0));
  const State reference = MakeReference(1.0, Eigen::Vector3d(2.0, 0.0, 0.0));
  const Cloud cloud = MakeCloud({0.1});

  const Cloud out = Deskew(cloud, trajectory, reference, Sophus::SE3d{});

  EXPECT_LT((out.points[0].position - cloud.points[0].position).norm(), 1e-12);
}

TEST(Deskew, PureRotationUndoesResidualYaw) {
  const double yaw_rate = 1.0;
  const Trajectory trajectory =
      MakeTrajectory(yaw_rate, Eigen::Vector3d::Zero());
  const State reference = MakeReference(yaw_rate, Eigen::Vector3d::Zero());
  const Cloud cloud = MakeCloud({0.0, 0.05});

  const Cloud out = Deskew(cloud, trajectory, reference, Sophus::SE3d{});

  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const double residual = yaw_rate * (cloud.points[i].timestamp - 0.1);
    const Eigen::Vector3d expected =
        Sophus::SO3d::rotZ(residual) * cloud.points[i].position;
    EXPECT_LT((out.points[i].position - expected).norm(), 1e-12);
  }
}

TEST(Deskew, PureTranslationUndoesResidualMotion) {
  const Eigen::Vector3d velocity(2.0, 0.0, 0.0);
  const Trajectory trajectory = MakeTrajectory(0.0, velocity);
  const State reference = MakeReference(0.0, velocity);
  const Cloud cloud = MakeCloud({0.0, 0.05});

  const Cloud out = Deskew(cloud, trajectory, reference, Sophus::SE3d{});

  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const Eigen::Vector3d expected =
        cloud.points[i].position + velocity * (cloud.points[i].timestamp - 0.1);
    EXPECT_LT((out.points[i].position - expected).norm(), 1e-12);
  }
}

TEST(Deskew, OutputIsInImuFrame) {
  const Trajectory trajectory = MakeTrajectory(0.0, Eigen::Vector3d::Zero());
  const State reference = MakeReference(0.0, Eigen::Vector3d::Zero());
  const Cloud cloud = MakeCloud({0.05});

  const Sophus::SE3d imu_from_lidar(Sophus::SO3d{},
                                    Eigen::Vector3d(-0.05, 0.0, 0.055));
  const Cloud out = Deskew(cloud, trajectory, reference, imu_from_lidar);

  const Eigen::Vector3d expected =
      cloud.points[0].position + Eigen::Vector3d(-0.05, 0.0, 0.055);
  EXPECT_LT((out.points[0].position - expected).norm(), 1e-12);
}

TEST(Deskew, ClampsOutsideTrajectoryRange) {
  const double yaw_rate = 1.0;
  const Trajectory trajectory =
      MakeTrajectory(yaw_rate, Eigen::Vector3d::Zero());
  const State reference = MakeReference(yaw_rate, Eigen::Vector3d::Zero());
  const Cloud cloud = MakeCloud({-1.0, 0.0, 0.1, 5.0});

  const Cloud out = Deskew(cloud, trajectory, reference, Sophus::SE3d{});

  EXPECT_LT((out.points[0].position - out.points[1].position).norm(), 1e-12);
  EXPECT_LT((out.points[3].position - out.points[2].position).norm(), 1e-12);
}

TEST(Deskew, PreservesPerPointMetadata) {
  const Trajectory trajectory = MakeTrajectory(1.0, Eigen::Vector3d::Zero());
  const State reference = MakeReference(1.0, Eigen::Vector3d::Zero());
  const Cloud cloud = MakeCloud({0.0, 0.05, 0.1});

  const Cloud out = Deskew(cloud, trajectory, reference, Sophus::SE3d{});

  ASSERT_EQ(out.points.size(), cloud.points.size());
  EXPECT_DOUBLE_EQ(out.start_time, cloud.start_time);
  EXPECT_DOUBLE_EQ(out.end_time, cloud.end_time);
  for (std::size_t i = 0; i < out.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(out.points[i].timestamp, cloud.points[i].timestamp);
    EXPECT_FLOAT_EQ(out.points[i].intensity, cloud.points[i].intensity);
  }
}

}  // namespace
}  // namespace livo_slam
