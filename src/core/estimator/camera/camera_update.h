#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"
#include "core/config/update_params.h"
#include "core/types/cloud.h"
#include "core/types/estimate.h"
#include "core/types/visual_observation.h"
#include "core/types/visual_point.h"

namespace livo_slam {

// TEMP DEBUG
struct CameraIterationLog {
  std::size_t level = 0;
  std::size_t iteration = 0;
  std::size_t matches = 0;
  double mean_cost = 0.0;
};

struct CameraUpdateResult {
  Estimate estimate;
  std::vector<VisualObservation> observations;
  std::vector<CameraIterationLog> iterations;  // TEMP DEBUG
  std::size_t applied_steps = 0;               // TEMP DEBUG
  std::size_t rejected_steps = 0;              // TEMP DEBUG
};

CameraUpdateResult UpdateWithCamera(
    const Estimate& prior, const std::vector<cv::Mat>& image_pyramid,
    const std::vector<const VisualPoint*>& visual_points,
    const std::vector<Point>& scan_imu, const CameraParams& camera_params,
    const CameraUpdateParams& update_params);
}  // namespace livo_slam
