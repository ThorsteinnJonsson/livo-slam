add_library(livo_slam_warnings INTERFACE)
target_compile_options(
  livo_slam_warnings
  INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Wthread-safety
    -Werror=thread-safety)
