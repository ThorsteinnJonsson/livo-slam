#include "core/pipeline/measurement_buffer.h"

#include <cstddef>
#include <iterator>
#include <utility>

namespace livo_slam {
namespace {

constexpr std::size_t kMaxImu = 4000;
constexpr std::size_t kMaxPoints = 1000000;
constexpr std::size_t kMaxImages = 50;

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
  {
    const MutexLock lock(mutex_);
    points_.insert(points_.end(), std::make_move_iterator(points.begin()),
                   std::make_move_iterator(points.end()));
    while (points_.size() > kMaxPoints) {
      points_.pop_front();
    }
  }
  cv_.notify_one();
}

void MeasurementBuffer::PushImage(ImageFrame image) {
  {
    const MutexLock lock(mutex_);
    images_.push_back(std::move(image));
    while (images_.size() > kMaxImages) {
      images_.pop_front();
    }
  }
  cv_.notify_one();
}

std::optional<MeasureGroup> MeasurementBuffer::PopMeasureGroup() {
  MutexLock lock(mutex_);
  while (!shutdown_ && !CanFormGroup()) {
    cv_.wait(lock.native());
  }
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
  return group;
}

}  // namespace livo_slam
