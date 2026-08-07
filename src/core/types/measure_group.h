#pragma once

#include <vector>

#include "core/types/cloud.h"
#include "core/types/image_frame.h"
#include "core/types/imu_sample.h"

namespace livo_slam {

struct MeasureGroup {
  ImageFrame image;
  Cloud cloud;
  std::vector<ImuSample> imu;
};

}  // namespace livo_slam
