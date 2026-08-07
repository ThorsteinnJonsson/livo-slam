#pragma once

#include <condition_variable>
#include <deque>
#include <optional>

#include "core/types/cloud.h"
#include "core/types/image_frame.h"
#include "core/types/imu_sample.h"
#include "core/types/measure_group.h"
#include "core/util/mutex.h"

namespace livo_slam {

class MeasurementBuffer {
 public:
  MeasurementBuffer() = default;

  MeasurementBuffer(const MeasurementBuffer&) = delete;
  MeasurementBuffer& operator=(const MeasurementBuffer&) = delete;
  MeasurementBuffer(MeasurementBuffer&&) = delete;
  MeasurementBuffer& operator=(MeasurementBuffer&&) = delete;

  void PushImu(ImuSample sample);
  void PushCloud(Cloud cloud);
  void PushImage(ImageFrame image);

  std::optional<MeasureGroup> PopMeasureGroup();

  void Shutdown();

 private:
  bool CanFormGroup() const REQUIRES(mutex_);
  MeasureGroup FormGroup() REQUIRES(mutex_);

 private:
  mutable Mutex mutex_;
  std::condition_variable cv_;
  bool shutdown_ GUARDED_BY(mutex_) = false;

  std::deque<ImuSample> imu_ GUARDED_BY(mutex_);
  std::deque<Cloud> clouds_ GUARDED_BY(mutex_);
  std::deque<ImageFrame> images_ GUARDED_BY(mutex_);
};

}  // namespace livo_slam
