#pragma once

#include "core/types/cloud.h"

namespace livo_slam {

Cloud FilterByRange(const Cloud& cloud, double min_range, double max_range);

}  // namespace livo_slam
