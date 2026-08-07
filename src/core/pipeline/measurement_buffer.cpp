#include "core/pipeline/measurement_buffer.h"

#include <cstddef>
#include <utility>

namespace livo_slam {
namespace {

constexpr std::size_t kMaxImu = 4000;
constexpr std::size_t kMaxClouds = 50;
constexpr std::size_t kMaxImages = 50;

}  // namespace

void MeasurementBuffer::PushImu(ImuSample sample) {
  {
    const MutexLock lock(mutex_);
    imu_.push_back(std::move(sample));
    if (imu_.size() > kMaxImu) {
      imu_.pop_front();
    }
  }
  cv_.notify_one();
}

void MeasurementBuffer::PushCloud(Cloud cloud) {
  {
    const MutexLock lock(mutex_);
    clouds_.push_back(std::move(cloud));
    if (clouds_.size() > kMaxClouds) {
      clouds_.pop_front();
    }
  }
  cv_.notify_one();
}

void MeasurementBuffer::PushImage(ImageFrame image) {
  {
    const MutexLock lock(mutex_);
    images_.push_back(std::move(image));
    if (images_.size() > kMaxImages) {
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
  return !clouds_.empty() && !imu_.empty() &&
         imu_.back().timestamp >= clouds_.front().end_time;
}

MeasureGroup MeasurementBuffer::FormGroup() {
  MeasureGroup group;
  group.cloud = std::move(clouds_.front());
  clouds_.pop_front();

  while (!imu_.empty() && imu_.front().timestamp <= group.cloud.end_time) {
    group.imu.push_back(std::move(imu_.front()));
    imu_.pop_front();
  }

  while (!images_.empty() &&
         images_.front().timestamp <= group.cloud.end_time) {
    group.image = std::move(images_.front());
    images_.pop_front();
  }

  return group;
}

}  // namespace livo_slam
