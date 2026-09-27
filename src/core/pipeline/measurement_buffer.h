#pragma once

#include <condition_variable>
#include <deque>
#include <optional>
#include <vector>

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
  void PushPoints(std::vector<Point> points);
  void PushImage(ImageFrame image);

  std::optional<MeasureGroup> PopMeasureGroup();

  // Returns a group only if one can be formed right now, without waiting for
  // more measurements. Used when a bag drives the pipeline directly.
  std::optional<MeasureGroup> TryPopMeasureGroup();

  void Shutdown();

 private:
  // Images older than the oldest point can never be grouped, since
  // FormGroup needs points on both sides of the image time.
  void DropStaleImages() REQUIRES(mutex_);
  bool CanFormGroup() const REQUIRES(mutex_);
  MeasureGroup FormGroup() REQUIRES(mutex_);

 private:
  mutable Mutex mutex_;
  std::condition_variable cv_;
  bool shutdown_ GUARDED_BY(mutex_) = false;
  std::optional<double> last_cut_time_ GUARDED_BY(mutex_);

  std::deque<ImuSample> imu_ GUARDED_BY(mutex_);
  std::deque<Point> points_ GUARDED_BY(mutex_);
  std::deque<ImageFrame> images_ GUARDED_BY(mutex_);
};

}  // namespace livo_slam
