#include "visual_point_selection.h"

#include <Eigen/Core>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <opencv2/core/mat.hpp>

#include "core/camera/projection.h"
#include "core/types/cloud.h"
#include "core/types/visual_point.h"

namespace livo_slam {

namespace {
struct Cell {
  bool covered = false;
  const Point* candidate = nullptr;
  double score = 0.0;
  Eigen::Vector2d pixel = Eigen::Vector2d::Zero();
  const VisualPoint* visual_point = nullptr;
  double distance = std::numeric_limits<double>::max();
};
}  // namespace

VisualMapUpdate SelectVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    double timestamp, const State& state, const CameraParams& camera_params,
    const CameraUpdateParams& camera_update_params,
    const VisualMapParams& visual_map_params,
    const std::vector<const VisualPoint*>& existing_visual_points) {
  VisualMapUpdate update;

  const Sophus::SE3d T_world_imu(state.rotation, state.position);
  const Sophus::SE3d T_world_camera = T_world_imu * camera_params.T_imu_cam;
  const Sophus::SE3d T_camera_world = T_world_camera.inverse();

  // Patch margin needs to cover the deepest level of the image pyramid.
  const std::size_t patch_margin =
      (camera_update_params.patch_size / 2 + 1) *
      (std::size_t{1} << (CameraUpdateParams::kPyramidLevels - 1));

  // Full-resolution image is split into a grid of cells.
  // New visual points will be generated in empty cells.
  // Cells are grid_size-pixel squares of the image.
  const std::size_t grid_size = visual_map_params.grid_size;
  const auto& full_res_image = image_pyramid[0];
  const std::size_t cols =
      (static_cast<std::size_t>(full_res_image.cols) + grid_size - 1) /
      grid_size;
  const std::size_t rows =
      (static_cast<std::size_t>(full_res_image.rows) + grid_size - 1) /
      grid_size;
  std::vector<Cell> cells(rows * cols);

  // Mark cells which already have a visual point projecting into them.
  for (const VisualPoint* visual_point : existing_visual_points) {
    // Project visual point into pixel
    const Eigen::Vector3d point_camera =
        T_camera_world * visual_point->position;
    const auto projected = Project(camera_params, point_camera);

    if (!projected || !InFrame(full_res_image, *projected, patch_margin)) {
      continue;
    }

    // Mark covered
    const auto pixel = projected->cast<std::size_t>();
    const std::size_t flat_index =
        pixel.y() / grid_size * cols + pixel.x() / grid_size;
    cells[flat_index].covered = true;
    if (cells[flat_index].distance > point_camera.norm()) {
      cells[flat_index].visual_point = visual_point;
      cells[flat_index].distance = point_camera.norm();
      cells[flat_index].pixel = *projected;
    }
  }

  // Find candidate point from lidar points.
  for (const Point& point : points) {
    // Project visual point into pixel
    const Eigen::Vector3d point_camera = T_camera_world * point.position;
    const auto projected = Project(camera_params, point_camera);

    if (!projected || !InFrame(full_res_image, *projected, patch_margin)) {
      continue;
    }
    const auto pixel = projected->cast<std::size_t>();
    const std::size_t flat_index =
        pixel.y() / grid_size * cols + pixel.x() / grid_size;
    if (cells[flat_index].covered) {
      continue;
    }

    const double corner_score = CornerScore(full_res_image, *projected);
    if (cells[flat_index].score < corner_score) {
      cells[flat_index].score = corner_score;
      cells[flat_index].candidate = &point;
      cells[flat_index].pixel = *projected;
    }
  }

  // Create new patches for existing visual points
  for (const Cell& cell : cells) {
    if (!cell.covered) {
      continue;
    }

    assert(!cell.visual_point->patches.empty());
    const VisualPatch& last_patch = cell.visual_point->patches.back();
    const bool stale = timestamp - last_patch.timestamp >
                       visual_map_params.min_seconds_between_patches;
    const bool moved = (cell.pixel - last_patch.pixel).norm() >
                       visual_map_params.min_patch_pixel_distance;
    if (!stale && !moved) {
      continue;
    }

    update.new_patches.emplace_back(
        cell.visual_point->position,
        CreateVisualPatch(image_pyramid, cell.pixel, T_world_camera, timestamp,
                          state.inv_exposure, camera_update_params.patch_size));
  }

  // Create new visual points
  for (const Cell& cell : cells) {
    if (cell.covered) {
      continue;
    }

    if (!cell.candidate || cell.score < visual_map_params.min_corner_score) {
      continue;
    }

    VisualPoint visual_point;
    visual_point.position = cell.candidate->position;
    visual_point.patches.push_back(
        CreateVisualPatch(image_pyramid, cell.pixel, T_world_camera, timestamp,
                          state.inv_exposure, camera_update_params.patch_size));
    update.new_points.push_back(std::move(visual_point));
  }

  return update;
}

VisualPatch CreateVisualPatch(const std::vector<cv::Mat>& image_pyramid,
                              const Eigen::Vector2d& pixel,
                              const Sophus::SE3d& T_world_camera,
                              double timestamp, double inv_exposure,
                              std::size_t patch_size) {
  VisualPatch patch;
  patch.pixel = pixel;
  patch.timestamp = timestamp;
  patch.inv_exposure = inv_exposure;
  patch.T_world_camera = T_world_camera;

  const int half_patch = static_cast<int>(patch_size) / 2;
  for (std::size_t level = CameraUpdateParams::kPyramidLevels; level-- > 0;) {
    const double scale = 1.0 / static_cast<double>(std::size_t{1} << level);
    const Eigen::Vector2d scaled_pixel = scale * pixel;

    const cv::Mat& image = image_pyramid[level];

    patch.intensities[level].resize(
        static_cast<Eigen::Index>(patch_size * patch_size));
    Eigen::Index index = 0;
    for (int dv = -half_patch; dv < half_patch; ++dv) {
      for (int du = -half_patch; du < half_patch; ++du) {
        const Eigen::Vector2d sample_pixel =
            scaled_pixel + Eigen::Vector2d(du, dv);
        patch.intensities[level][index] =
            static_cast<float>(SampleBilinear(image, sample_pixel));
        ++index;
      }
    }
  }

  return patch;
}

double CornerScore(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  constexpr int kCornerWindow = 8;
  constexpr int kCornerHalfWindow = kCornerWindow / 2;
  constexpr double kNormalizer = 2.0 * kCornerWindow * kCornerWindow;

  const int u = static_cast<int>(pixel.x());
  const int v = static_cast<int>(pixel.y());
  double dxx = 0.0;
  double dyy = 0.0;
  double dxy = 0.0;
  for (int dv = -kCornerHalfWindow; dv < kCornerHalfWindow; ++dv) {
    for (int du = -kCornerHalfWindow; du < kCornerHalfWindow; ++du) {
      const int c = u + du;
      const int r = v + dv;
      const double dx =
          image.at<std::uint8_t>(r, c + 1) - image.at<std::uint8_t>(r, c - 1);
      const double dy =
          image.at<std::uint8_t>(r + 1, c) - image.at<std::uint8_t>(r - 1, c);
      dxx += dx * dx;
      dyy += dy * dy;
      dxy += dx * dy;
    }
  }

  dxx /= kNormalizer;
  dyy /= kNormalizer;
  dxy /= kNormalizer;

  return 0.5 * (dxx + dyy - std::sqrt(std::pow(dxx - dyy, 2) + 4 * dxy * dxy));
}

}  // namespace livo_slam