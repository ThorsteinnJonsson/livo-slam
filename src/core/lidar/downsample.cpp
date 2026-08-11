#include "downsample.h"

#include <cstdint>
#include <unordered_map>

#include "core/math/voxel_key.h"

namespace livo_slam {
namespace {

struct VoxelAccumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  double sum_intensity = 0.0;
  double earliest_timestamp = 0.0;
  std::uint32_t count = 0;
};

}  // namespace

Cloud VoxelDownsample(const Cloud& cloud, double leaf_size) {
  Cloud downsampled;
  downsampled.start_time = cloud.start_time;
  downsampled.end_time = cloud.end_time;
  if (leaf_size <= 0.0) {
    return cloud;
  }

  std::unordered_map<std::int64_t, VoxelAccumulator> grid;
  grid.reserve(cloud.points.size());

  for (const Point& point : cloud.points) {
    if (!point.position.allFinite()) {
      continue;
    }

    auto& accumulator =
        grid[PackKey(VoxelIndex(point.position, leaf_size))];
    accumulator.sum += point.position;
    accumulator.sum_intensity += static_cast<double>(point.intensity);
    if (accumulator.count == 0 ||
        point.timestamp < accumulator.earliest_timestamp) {
      accumulator.earliest_timestamp = point.timestamp;
    }
    ++accumulator.count;
  }

  downsampled.points.reserve(grid.size());
  for (const auto& [key, accumulator] : grid) {
    const double count = static_cast<double>(accumulator.count);

    Point point;
    point.position = accumulator.sum / count;
    point.intensity = static_cast<float>(accumulator.sum_intensity / count);
    point.timestamp = accumulator.earliest_timestamp;
    downsampled.points.push_back(point);
  }

  return downsampled;
}

}  // namespace livo_slam
