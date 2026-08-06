# Selected through CMakePresets `toolchainFile`. This has to be a toolchain file
# rather than add_compile_options: only a toolchain propagates into FetchContent
# and ExternalProject sub-builds, so dependencies compile against the same
# standard library we do.

set(CMAKE_C_COMPILER clang-22 CACHE FILEPATH "")
set(CMAKE_CXX_COMPILER clang++-22 CACHE FILEPATH "")

if(DEFINED ENV{GCC_INSTALL_DIR})
  set(LIVO_SLAM_GCC_INSTALL_DIR "$ENV{GCC_INSTALL_DIR}")
else()
  set(LIVO_SLAM_GCC_INSTALL_DIR "/usr/lib/gcc/x86_64-linux-gnu/14")
endif()

# clang picks the newest GCC installation it finds, which on noble is 13. That
# libstdc++ has no <print>, so pin the version instead of taking the default.
set(CMAKE_C_FLAGS_INIT "--gcc-install-dir=${LIVO_SLAM_GCC_INSTALL_DIR}")
set(CMAKE_CXX_FLAGS_INIT "--gcc-install-dir=${LIVO_SLAM_GCC_INSTALL_DIR}")

# Only the versioned lld is installed, so -fuse-ld=lld would not resolve.
set(LIVO_SLAM_LINK_FLAGS
    "--gcc-install-dir=${LIVO_SLAM_GCC_INSTALL_DIR} --ld-path=/usr/bin/ld.lld-22")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${LIVO_SLAM_LINK_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${LIVO_SLAM_LINK_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${LIVO_SLAM_LINK_FLAGS}")
