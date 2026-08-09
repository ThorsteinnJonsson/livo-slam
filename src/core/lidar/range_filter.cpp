#include "range_filter.h"

namespace livo_slam {

Cloud FilterByRange(const Cloud& cloud, double min_range, double max_range) {
  Cloud filtered;
  filtered.start_time = cloud.start_time;
  filtered.end_time = cloud.end_time;
  filtered.points.reserve(cloud.points.size());

  const double min_squared = min_range * min_range;
  const double max_squared = max_range * max_range;

  for (const Point& point : cloud.points) {
    const double range_squared = point.position.squaredNorm();
    if (range_squared >= min_squared && range_squared <= max_squared) {
      filtered.points.push_back(point);
    }
  }
  filtered.points.shrink_to_fit();

  return filtered;
}

}  // namespace livo_slam
