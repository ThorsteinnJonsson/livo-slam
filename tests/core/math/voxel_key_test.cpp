#include "core/math/voxel_key.h"

#include <gtest/gtest.h>

#include <set>

namespace livo_slam::voxel {
namespace {

TEST(UnpackKey, RecoversTheIndex) {
  for (const Eigen::Vector3i index :
       {Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(1, -2, 3),
        Eigen::Vector3i(-1000, 2000, -3000)}) {
    EXPECT_EQ(UnpackKey(PackKey(index)), index);
  }
}

TEST(UnpackKey, RecoversTheIndexAtTheRangeLimits) {
  const int limit = (1 << 20) - 1;
  EXPECT_EQ(UnpackKey(PackKey({limit, -limit, limit})),
            Eigen::Vector3i(limit, -limit, limit));
}

TEST(VoxelIndex, FloorsTowardsNegativeInfinity) {
  EXPECT_EQ(VoxelIndex({-0.4, -0.6, -1.4}, 1.0), Eigen::Vector3i(-1, -1, -2));
}

TEST(VoxelIndex, ScalesByVoxelSize) {
  EXPECT_EQ(VoxelIndex({0.1, 0.6, 1.2}, 0.5), Eigen::Vector3i(0, 1, 2));
  EXPECT_EQ(VoxelIndex({0.1, 0.6, 1.2}, 1.0), Eigen::Vector3i(0, 0, 1));
}

TEST(VoxelIndex, BoundaryBelongsToTheHigherVoxel) {
  EXPECT_EQ(VoxelIndex({1.0, 0.0, -1.0}, 1.0), Eigen::Vector3i(1, 0, -1));
}

TEST(VoxelIndex, PointsInOneVoxelShareAnIndex) {
  EXPECT_EQ(VoxelIndex({0.01, 0.49, 0.25}, 0.5),
            VoxelIndex({0.49, 0.01, 0.02}, 0.5));
}

TEST(PackKey, IsDeterministic) {
  EXPECT_EQ(PackKey({5, -7, 9}), PackKey({5, -7, 9}));
}

TEST(PackKey, SeparatesTheAxes) {
  EXPECT_NE(PackKey({1, 0, 0}), PackKey({0, 1, 0}));
  EXPECT_NE(PackKey({1, 0, 0}), PackKey({0, 0, 1}));
  EXPECT_NE(PackKey({0, 1, 0}), PackKey({0, 0, 1}));
}

TEST(PackKey, DistinguishesNegativeFromPositive) {
  EXPECT_NE(PackKey({-1, -1, -1}), PackKey({1, 1, 1}));
  EXPECT_NE(PackKey({-1, 0, 0}), PackKey({1, 0, 0}));
}

TEST(PackKey, IsUniqueOverANeighbourhood) {
  std::set<std::int64_t> keys;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      for (int z = -3; z <= 3; ++z) {
        keys.insert(PackKey({x, y, z}));
      }
    }
  }

  EXPECT_EQ(keys.size(), 7U * 7U * 7U);
}

TEST(PackKey, IsUniqueAcrossTheRepresentableRange) {
  constexpr int kLimit = (1 << 20) - 1;
  std::set<std::int64_t> keys;
  for (int value : {-kLimit, -1, 0, 1, kLimit}) {
    keys.insert(PackKey({value, 0, 0}));
    keys.insert(PackKey({0, value, 0}));
    keys.insert(PackKey({0, 0, value}));
  }

  EXPECT_EQ(keys.size(), 13U);
}

}  // namespace
}  // namespace livo_slam::voxel
