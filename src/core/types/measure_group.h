#pragma once

#include <vector>

#include "core/types/cloud.h"
#include "core/types/image_frame.h"
#include "core/types/imu_sample.h"

namespace livo_slam {

struct MeasureGroup {
  Cloud cloud;
  std::vector<ImuSample> imu;
  ImageFrame image;
};

}  // namespace livo_slam
