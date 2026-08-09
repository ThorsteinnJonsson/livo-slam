#pragma once

#include "core/types/cloud.h"

namespace livo_slam {

Cloud VoxelDownsample(const Cloud& cloud, double leaf_size);

}  // namespace livo_slam
