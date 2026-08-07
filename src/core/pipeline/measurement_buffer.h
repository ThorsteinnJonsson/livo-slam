#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

#include "core/types/cloud.h"
#include "core/types/image_frame.h"
#include "core/types/imu_sample.h"
#include "core/types/measure_group.h"

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
  bool CanFormGroup() const;
  MeasureGroup FormGroup();

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool shutdown_ = false;

  std::deque<ImuSample> imu_;
  std::deque<Cloud> clouds_;
  std::deque<ImageFrame> images_;
};

}  // namespace livo_slam
