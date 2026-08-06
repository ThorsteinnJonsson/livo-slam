include(FetchContent)

# Dependency test suites otherwise register themselves into our ctest run. Eigen
# alone contributes 900+ tests. Our own switch is LIVO_SLAM_BUILD_TESTS, kept
# deliberately distinct from BUILD_TESTING for exactly this reason.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# SYSTEM keeps dependency headers out of our warning output.
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
  SYSTEM)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# Sophus calls find_package(Eigen3), which would otherwise resolve to noble's
# system copy in /usr/share/eigen3 rather than the version pinned above.
set(Eigen3_DIR "${eigen_BINARY_DIR}" CACHE PATH "" FORCE)

FetchContent_Declare(
  sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG 1.24.6
  GIT_SHALLOW TRUE
  SYSTEM)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(sophus)

if(LIVO_SLAM_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
    SYSTEM)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()
