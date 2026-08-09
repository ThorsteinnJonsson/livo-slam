#pragma once

#include "core/types/cloud.h"
#include "core/types/estimate.h"
#include "core/types/trajectory.h"
namespace livo_slam {

Cloud Deskew(const Estimate& estimate, const Trajectory& trajectory,
             const Cloud& cloud, double end_time);

}  // namespace livo_slam