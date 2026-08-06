// Proves the pinned dependencies actually compile under clang-22 at C++23,
// which configuring alone does not establish for header-only libraries.
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <numbers>
#include <print>
#include <sophus/so3.hpp>

TEST(Smoke, EigenCompilesAndComputes) {
  const Eigen::Vector3d v{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(v.norm(), std::sqrt(14.0));
}

TEST(Smoke, SophusRotatesAboutZ) {
  const auto rotation =
      Sophus::SO3d::rotZ(std::numbers::pi / 2.0) * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(rotation.x(), 0.0, 1e-12);
  EXPECT_NEAR(rotation.y(), 1.0, 1e-12);
}

TEST(Smoke, StdLibraryIsLibstdcxx14) {
  EXPECT_GE(_GLIBCXX_RELEASE, 14);
  std::println("libstdc++ {}", _GLIBCXX_RELEASE);
}
