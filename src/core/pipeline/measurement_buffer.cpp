#include "core/pipeline/measurement_buffer.h"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <print>  // TEMP DEBUG
#include <utility>
#include <vector>

namespace livo_slam {
namespace {

constexpr std::size_t kMaxImu = 4000;
constexpr std::size_t kMaxPoints = 1000000;
constexpr std::size_t kMaxImages = 50;

// TEMP DEBUG
std::size_t g_images_pushed = 0;
std::size_t g_points_pushed = 0;
std::size_t g_stale_dropped = 0;
std::size_t g_cap_dropped_images = 0;
std::size_t g_cap_dropped_points = 0;
std::size_t g_groups = 0;

const auto by_time = [](const Point& lhs, const Point& rhs) {
  return lhs.timestamp < rhs.timestamp;
};

}  // namespace

void MeasurementBuffer::PushImu(ImuSample sample) {
  {
    const MutexLock lock(mutex_);
    imu_.push_back(std::move(sample));
    while (imu_.size() > kMaxImu) {
      imu_.pop_front();
    }
  }
  cv_.notify_one();
}

void MeasurementBuffer::PushPoints(std::vector<Point> points) {
  if (points.empty()) {
    return;
  }
  ++g_points_pushed;  // TEMP DEBUG

  {
    const MutexLock lock(mutex_);

    // Consecutive scans can overlap in absolute point time. In that case we
    // can't append, since we need the points to be sorted by timestamp.
    if (points_.empty() ||
        points_.back().timestamp <= points.front().timestamp) {
      points_.insert(points_.end(), std::make_move_iterator(points.begin()),
                     std::make_move_iterator(points.end()));
    } else {
      const auto split = std::lower_bound(points_.begin(), points_.end(),
                                          points.front(), by_time);

      std::vector<Point> overlap(std::make_move_iterator(split),
                                 std::make_move_iterator(points_.end()));
      points_.erase(split, points_.end());

      std::vector<Point> merged;
      merged.reserve(overlap.size() + points.size());
      std::merge(std::make_move_iterator(overlap.begin()),
                 std::make_move_iterator(overlap.end()),
                 std::make_move_iterator(points.begin()),
                 std::make_move_iterator(points.end()),
                 std::back_inserter(merged), by_time);

      points_.insert(points_.end(), std::make_move_iterator(merged.begin()),
                     std::make_move_iterator(merged.end()));
    }

    while (points_.size() > kMaxPoints) {
      points_.pop_front();
      ++g_cap_dropped_points;
    }
  }
  cv_.notify_one();
}

void MeasurementBuffer::PushImage(ImageFrame image) {
  {
    const MutexLock lock(mutex_);
    images_.push_back(std::move(image));
    ++g_images_pushed;
    while (images_.size() > kMaxImages) {
      images_.pop_front();
      ++g_cap_dropped_images;
    }
  }
  cv_.notify_one();
}

std::optional<MeasureGroup> MeasurementBuffer::PopMeasureGroup() {
  MutexLock lock(mutex_);
  DropStaleImages();
  while (!shutdown_ && !CanFormGroup()) {
    cv_.wait(lock.native());
    DropStaleImages();
  }
  if (!CanFormGroup()) {
    return std::nullopt;
  }
  return FormGroup();
}

std::optional<MeasureGroup> MeasurementBuffer::TryPopMeasureGroup() {
  MutexLock lock(mutex_);
  DropStaleImages();
  if (!CanFormGroup()) {
    return std::nullopt;
  }
  return FormGroup();
}

void MeasurementBuffer::Shutdown() {
  {
    const MutexLock lock(mutex_);
    shutdown_ = true;
  }
  cv_.notify_all();
}

void MeasurementBuffer::DropStaleImages() {
  while (!images_.empty() && !points_.empty() &&
         images_.front().timestamp < points_.front().timestamp) {
    // TEMP DEBUG
    std::println(
        "STALE_DROP image_t={:.4f} points_front={:.4f} behind={:.4f} "
        "images={} points={}",
        images_.front().timestamp, points_.front().timestamp,
        points_.front().timestamp - images_.front().timestamp, images_.size(),
        points_.size());
    ++g_stale_dropped;
    images_.pop_front();
  }
}

bool MeasurementBuffer::CanFormGroup() const {
  if (images_.empty() || points_.empty() || imu_.empty()) {
    return false;
  }
  const double image_time = images_.front().timestamp;
  return points_.front().timestamp <= image_time &&
         points_.back().timestamp >= image_time &&
         imu_.back().timestamp >= image_time;
}

MeasureGroup MeasurementBuffer::FormGroup() {
  MeasureGroup group;
  group.image = std::move(images_.front());
  images_.pop_front();

  const double image_time = group.image.timestamp;
  group.cloud.start_time = last_cut_time_.value_or(points_.front().timestamp);
  group.cloud.end_time = image_time;

  while (!points_.empty() && points_.front().timestamp <= image_time) {
    group.cloud.points.push_back(points_.front());
    points_.pop_front();
  }

  while (!imu_.empty() && imu_.front().timestamp <= image_time) {
    group.imu.push_back(std::move(imu_.front()));
    imu_.pop_front();
  }

  last_cut_time_ = image_time;
  // TEMP DEBUG
  if (++g_groups % 500 == 0) {
    std::println(
        "BUFSTATS groups={} images_pushed={} scans_pushed={} stale_dropped={} "
        "cap_dropped_images={} cap_dropped_points={}",
        g_groups, g_images_pushed, g_points_pushed, g_stale_dropped,
        g_cap_dropped_images, g_cap_dropped_points);
  }
  return group;
}

}  // namespace livo_slam
