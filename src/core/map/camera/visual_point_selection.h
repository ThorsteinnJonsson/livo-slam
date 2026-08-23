#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"
#include "core/config/map_params.h"
#include "core/types/cloud.h"
#include "core/types/state.h"
#include "core/types/visual_point.h"

namespace livo_slam {

struct VisualMapUpdate {
  std::vector<VisualPoint> new_points;
  std::vector<std::pair<Eigen::Vector3d, VisualPatch>> new_patches;
};

VisualMapUpdate SelectVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    double timestamp, const State& state, const CameraParams& camera,
    const CameraUpdateParams& camera_update_params,
    const VisualMapParams& visual_map_params, const VoxelMapParams& map_params,
    const std::vector<const VisualPoint*>& existing_visual_points);

VisualPatch CreateVisualPatch(const std::vector<cv::Mat>& image_pyramid,
                              const Eigen::Vector2d& pixel,
                              const Sophus::SE3d& T_world_camera,
                              double inv_exposure, std::size_t patch_size);

double CornerScore(const cv::Mat& image, const Eigen::Vector2d& pixel);

}  // namespace livo_slam