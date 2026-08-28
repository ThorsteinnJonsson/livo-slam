#include "visual_point_selection.h"

#include <cstddef>
#include <limits>

#include "core/camera/projection.h"
#include "core/types/visual_point.h"

namespace livo_slam {

namespace {
struct Cell {
  bool covered = false;
  const Point* candidate = nullptr;
  double score = 0.0;
};
}  // namespace

VisualMapUpdate SelectVisualPoints(
    const std::vector<Point>& points, const std::vector<cv::Mat>& image_pyramid,
    double timestamp, const State& state, const CameraParams& camera_params,
    const CameraUpdateParams& camera_update_params,
    const VisualMapParams& visual_map_params, const VoxelMapParams& map_params,
    const std::vector<const VisualPoint*>& existing_visual_points) {
  // Cells are grid_size-pixel squares of the image.
  //
  // For each existing visual point
  //    Project, skip if behind or not InFrame with a patch_size margin
  //    Mark its cell as covered, and remember the point and its pixel
  //
  // For each candidate point
  //    Project, skip if behind or not InFrame with a patch_size margin
  //    Skip if its cell is covered
  //    Keep it as the cell's candidate if its CornerScore is the highest so
  //    far, and its depth the smallest of the candidates in its voxel
  //
  // For each covered cell
  //    Add a patch for its visual point if more than
  //    min_seconds_between_patches have passed since its last patch, or the
  //    projection has moved further than min_patch_pixel_distance from where
  //    that patch was taken
  //
  // For each cell with a candidate
  //    Skip if the score is below the minimum
  //    CreateVisualPatch at the projected pixel
  //    Append a VisualPoint holding that patch

  const Sophus::SE3d T_world_imu(state.rotation, state.position);
  const Sophus::SE3d T_world_camera = T_world_imu * camera_params.T_imu_cam;
  const Sophus::SE3d T_camera_world = T_world_camera.inverse();

  const std::size_t patch_margin = camera_update_params.patch_size / 2 + 1;

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
  }

  return {};  // TODO: Implement
}

VisualPatch CreateVisualPatch(const std::vector<cv::Mat>& image_pyramid,
                              const Eigen::Vector2d& pixel,
                              const Sophus::SE3d& T_world_camera,
                              double inv_exposure, std::size_t patch_size) {
  return {};  // TODO: Implement
}

double CornerScore(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  return std::numeric_limits<double>::quiet_NaN();  // TODO: Implement
}

}  // namespace livo_slam