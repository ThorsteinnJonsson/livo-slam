#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"
#include "core/config/update_params.h"
#include "core/types/estimate.h"
#include "core/types/visual_observation.h"
#include "core/types/visual_point.h"

namespace livo_slam {

struct CameraUpdateResult {
  Estimate estimate;
  std::vector<VisualObservation> observations;
};

CameraUpdateResult UpdateWithCamera(
    const Estimate& prior, const std::vector<cv::Mat>& image_pyramid,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params, const CameraUpdateParams& update_params);
}  // namespace livo_slam
