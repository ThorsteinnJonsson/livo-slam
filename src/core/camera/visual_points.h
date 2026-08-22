#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"
#include "core/map/voxel_map.h"
#include "core/types/cloud.h"
#include "core/types/state.h"
#include "core/types/visual_point.h"

namespace livo_slam {

std::vector<VisualPoint> BuildVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    const State& state, const CameraParams& camera,
    const CameraUpdateParams& params, const VoxelMap& map);

VisualPatch CreateVisualPatch(const std::vector<cv::Mat>& image_pyramid,
                              const Eigen::Vector2d& pixel,
                              const Sophus::SE3d& T_world_camera,
                              double inv_exposure, std::size_t patch_size);

double CornerScore(const cv::Mat& image, const Eigen::Vector2d& pixel);

}  // namespace livo_slam