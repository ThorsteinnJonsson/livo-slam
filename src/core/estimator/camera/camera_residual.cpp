#include "camera_residual.h"

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

#include "core/camera/projection.h"
#include "core/math/skew.h"
#include "core/types/state.h"
#include "core/types/visual_point.h"

namespace livo_slam {

const VisualPatch* SelectReferencePatch(
    const VisualPoint& visual_point, const Eigen::Vector3d& camera_position) {
  const Eigen::Vector3d view_dir =
      (camera_position - visual_point.position).normalized();

  double max_cos_angle = std::numeric_limits<double>::lowest();
  const VisualPatch* best_match = nullptr;
  for (const VisualPatch& patch : visual_point.patches) {
    const Eigen::Vector3d patch_dir =
        (patch.T_world_camera.translation() - visual_point.position)
            .normalized();
    const double cos_angle = view_dir.dot(patch_dir);
    if (best_match == nullptr || max_cos_angle < cos_angle) {
      max_cos_angle = cos_angle;
      best_match = &patch;
    }
  }
  return best_match;
}

PhotometricMatch CreatePhotometricMatch(const cv::Mat& image,
                                        const Eigen::Vector2d& pixel,
                                        const Eigen::Vector3d& point_imu,
                                        const VisualPatch& reference_patch,
                                        std::size_t pyramid_level,
                                        std::size_t patch_size,
                                        const Eigen::Matrix2d& warp) {
  const int half_patch = static_cast<int>(patch_size) / 2;
  const Eigen::Index total_patch_size =
      static_cast<Eigen::Index>(patch_size * patch_size);

  PhotometricMatch match;
  match.point_imu = point_imu;
  match.reference_inv_exposure = reference_patch.inv_exposure;
  match.current_patch.resize(total_patch_size);
  match.gradients.resize(total_patch_size, 2);

  Eigen::Index index = 0;
  for (int dv = -half_patch; dv < half_patch; ++dv) {
    for (int du = -half_patch; du < half_patch; ++du) {
      const Eigen::Vector2d sample_pixel =
          pixel + warp * Eigen::Vector2d(du, dv);
      match.current_patch(index) =
          static_cast<float>(SampleBilinear(image, sample_pixel));
      match.gradients(index, 0) =
          0.5 *
          (SampleBilinear(image, sample_pixel + Eigen::Vector2d::UnitX()) -
           SampleBilinear(image, sample_pixel - Eigen::Vector2d::UnitX()));
      match.gradients(index, 1) =
          0.5 *
          (SampleBilinear(image, sample_pixel + Eigen::Vector2d::UnitY()) -
           SampleBilinear(image, sample_pixel - Eigen::Vector2d::UnitY()));
      ++index;
    }
  }

  match.reference_patch = reference_patch.intensities[pyramid_level];

  return match;
}

cv::Mat BuildDepthImage(cv::Size size, const std::vector<Point>& points_imu,
                        const CameraParams& camera_params) {
  cv::Mat depth_image = cv::Mat::zeros(size, CV_32F);
  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();

  for (const Point& point : points_imu) {
    const Eigen::Vector3d point_cam = T_cam_imu * point.position;
    if (point_cam.z() <= 0.0) continue;

    const std::optional<Eigen::Vector2d> projected =
        Project(camera_params, point_cam);
    if (!projected) continue;

    const int column = static_cast<int>(projected->x());
    const int row = static_cast<int>(projected->y());
    if (column < 0 || row < 0 || column >= size.width || row >= size.height) {
      continue;
    }

    // Keep the nearest, which is the surface the camera actually sees.
    float& stored = depth_image.at<float>(row, column);
    const auto depth = static_cast<float>(point_cam.z());
    if (stored == 0.0F || depth < stored) {
      stored = depth;
    }
  }
  return depth_image;
}

namespace {

// The nearest visible point in each grid cell, chosen before any gate runs.
// Keeps the nearest point in each grid cell, so the constraints are spread over
// the image instead of piling onto whatever happens to be textured.
std::vector<SelectedPoint> DecimateByGrid(
    const std::vector<SelectedPoint>& selection, const cv::Mat& image,
    const CameraUpdateParams& update_params, double scale) {
  const std::size_t grid_size = update_params.selection_grid_size;
  if (grid_size == 0) return selection;

  const std::size_t columns =
      (static_cast<std::size_t>(image.cols) + grid_size - 1) / grid_size;
  const std::size_t rows =
      (static_cast<std::size_t>(image.rows) + grid_size - 1) / grid_size;

  std::vector<int> best(rows * columns, -1);
  for (std::size_t i = 0; i < selection.size(); ++i) {
    const Eigen::Vector2d pixel = selection[i].pixel * scale;
    if (pixel.x() < 0.0 || pixel.y() < 0.0) continue;
    const auto column = static_cast<std::size_t>(pixel.x()) / grid_size;
    const auto row = static_cast<std::size_t>(pixel.y()) / grid_size;
    if (column >= columns || row >= rows) continue;

    int& winner = best[row * columns + column];
    if (winner < 0 ||
        selection[i].distance <
            selection[static_cast<std::size_t>(winner)].distance) {
      winner = static_cast<int>(i);
    }
  }

  std::vector<SelectedPoint> decimated;
  decimated.reserve(best.size());
  for (const int index : best) {
    if (index >= 0)
      decimated.push_back(selection[static_cast<std::size_t>(index)]);
  }
  return decimated;
}

// Samples reach warp * (half_patch, half_patch) from the center, and the
// gradient needs one pixel beyond that.
std::size_t WarpedMargin(const Eigen::Matrix2d& warp, double half_patch) {
  return static_cast<std::size_t>(
      warp.cwiseAbs().rowwise().sum().maxCoeff() * half_patch + 2.0);
}

// True when the current scan sees a surface inside the patch that is much
// nearer or further than the point itself, so the patch spans an edge.
bool OnDepthEdge(const cv::Mat& depth_image, const Eigen::Vector2d& pixel,
                 double depth, int half_patch, double max_discontinuity) {
  const int center_column = static_cast<int>(pixel.x());
  const int center_row = static_cast<int>(pixel.y());

  for (int dv = -half_patch; dv <= half_patch; ++dv) {
    const int row = center_row + dv;
    if (row < 0 || row >= depth_image.rows) continue;
    for (int du = -half_patch; du <= half_patch; ++du) {
      const int column = center_column + du;
      if (column < 0 || column >= depth_image.cols) continue;

      const float measured = depth_image.at<float>(row, column);
      if (measured == 0.0F) continue;  // Nothing hit this pixel.
      if (std::abs(depth - static_cast<double>(measured)) > max_discontinuity) {
        return true;
      }
    }
  }
  return false;
}

// A patch only stays usable while the warp keeps it close to its original
// scale; beyond this it is either aliased or blown up out of nothing.
constexpr double kMinWarpDeterminant = 1.0 / 16.0;
constexpr double kMaxWarpDeterminant = 16.0;

// Maps a pixel offset in the reference patch to the matching pixel offset in
// the current image. The offset is traced onto the surface plane through the
// point, so a slanted surface warps as a homography rather than as if it faced
// the reference camera. Both sides use the same pyramid level, so the matrix is
// the same at every level.
Eigen::Matrix2d AffineWarp(const CameraParams& camera_params,
                           const VisualPatch& reference_patch,
                           const Eigen::Vector3d& point_world,
                           const Eigen::Vector3d& normal_world,
                           const Eigen::Vector3d& point_cam,
                           const Sophus::SE3d& T_cam_world) {
  const Sophus::SE3d T_ref_world = reference_patch.T_world_camera.inverse();
  const Eigen::Vector3d point_ref = T_ref_world * point_world;
  if (point_ref.z() <= 0.0) {
    return Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
  }

  const Eigen::Vector3d normal_ref =
      (T_ref_world.so3() * normal_world).normalized();

  // A pixel offset fixes the two projection rows; staying on the plane fixes
  // the third. Solving all three turns the offset into a 3D offset along the
  // surface.
  Eigen::Matrix3d reference_map;
  reference_map.topRows<2>() = ProjectionJacobian(camera_params, point_ref);
  reference_map.row(2) = normal_ref.transpose();
  if (std::abs(reference_map.determinant()) <
      std::numeric_limits<double>::epsilon()) {
    return Eigen::Matrix2d::Constant(std::numeric_limits<double>::quiet_NaN());
  }

  const Eigen::Matrix3d R_cam_ref =
      (T_cam_world * reference_patch.T_world_camera).rotationMatrix();

  return ProjectionJacobian(camera_params, point_cam) * R_cam_ref *
         reference_map.inverse().leftCols<2>();
}

}  // namespace

std::vector<SelectedPoint> SelectCameraPoints(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<const VisualPoint*>& visual_points,
    const CameraParams& camera_params, const CameraUpdateParams& update_params,
    std::vector<VisualObservation>* observations, const cv::Mat& depth_image) {
  const double min_view_angle_cos =
      std::cos(update_params.max_view_angle_deg * std::numbers::pi / 180.0);
  const double scale =
      1.0 / static_cast<double>(std::size_t{1} << pyramid_level);

  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();
  const Sophus::SO3d R_inv = state.rotation.inverse();
  const Sophus::SE3d T_world_imu(state.rotation, state.position);
  const Sophus::SE3d T_world_cam = T_world_imu * camera_params.T_imu_cam;
  const Sophus::SE3d T_cam_world = T_world_cam.inverse();
  const Eigen::Vector3d cam_position = T_world_cam.translation();
  const double half_patch = static_cast<double>(update_params.patch_size) / 2.0;

  std::vector<SelectedPoint> selection;
  selection.reserve(visual_points.size());
  for (const VisualPoint* visual_point : visual_points) {
    const Eigen::Vector3d p_imu =
        R_inv * (visual_point->position - state.position);
    const Eigen::Vector3d p_cam = T_cam_imu * p_imu;

    const std::optional<Eigen::Vector2d> projected =
        Project(camera_params, p_cam);
    if (!projected) continue;

    // Scale the pixel to this pyramid level
    const Eigen::Vector2d pixel = (*projected) * scale;

    // Without a plane the warp has no surface to trace the patch onto.
    if (visual_point->normal.isZero()) {
      continue;
    }

    const VisualPatch* reference_patch =
        SelectReferencePatch(*visual_point, cam_position);

    if (!reference_patch) continue;  // Never observed

    {
      const Eigen::Vector3d view_dir =
          (cam_position - visual_point->position).normalized();
      const Eigen::Vector3d patch_dir =
          (reference_patch->T_world_camera.translation() -
           visual_point->position)
              .normalized();
      if (view_dir.dot(patch_dir) < min_view_angle_cos) continue;
    }

    if (!depth_image.empty() &&
        OnDepthEdge(depth_image, *projected, p_cam.z(),
                    static_cast<int>(update_params.patch_size) / 2,
                    update_params.max_depth_discontinuity)) {
      continue;
    }

    // The patch was taken from a different viewpoint, so the surface it sits
    // on projects into this frame stretched and rotated. Warping the sample
    // pattern by that map compares the same surface points instead of the same
    // pixel offsets.
    const Eigen::Matrix2d warp =
        AffineWarp(camera_params, *reference_patch, visual_point->position,
                   visual_point->normal, p_cam, T_cam_world);
    if (!warp.allFinite()) continue;

    const double determinant = warp.determinant();
    if (!(determinant > kMinWarpDeterminant &&
          determinant < kMaxWarpDeterminant)) {
      continue;  // Too stretched to carry usable detail.
    }

    if (!InFrame(image, pixel, WarpedMargin(warp, half_patch))) continue;

    PhotometricMatch match =
        CreatePhotometricMatch(image, pixel, p_imu, *reference_patch,
                               pyramid_level, update_params.patch_size, warp);

    const double photometric_error =
        (match.reference_inv_exposure * match.reference_patch -
         state.inv_exposure * match.current_patch)
            .squaredNorm();

    // The threshold is per pixel, the error is summed over the patch.
    const double outlier_threshold =
        update_params.outlier_threshold *
        static_cast<double>(update_params.patch_size *
                            update_params.patch_size);

    const bool matched = photometric_error < outlier_threshold;
    if (observations) {
      observations->push_back({visual_point->position, *projected, matched});
    }
    if (matched) {
      selection.push_back(
          {visual_point, reference_patch, warp, *projected, p_cam.norm()});
    }
  }

  selection = DecimateByGrid(selection, image, update_params, scale);
  selection.shrink_to_fit();
  return selection;
}

std::vector<PhotometricMatch> BuildMatchesFromSelection(
    const cv::Mat& image, std::size_t pyramid_level, const State& state,
    const std::vector<SelectedPoint>& selection,
    const CameraParams& camera_params,
    const CameraUpdateParams& update_params) {
  const double scale =
      1.0 / static_cast<double>(std::size_t{1} << pyramid_level);
  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();
  const Sophus::SO3d R_inv = state.rotation.inverse();
  const double half_patch = static_cast<double>(update_params.patch_size) / 2.0;

  std::vector<PhotometricMatch> matches;
  matches.reserve(selection.size());
  for (const SelectedPoint& selected : selection) {
    const Eigen::Vector3d p_imu =
        R_inv * (selected.point->position - state.position);
    const Eigen::Vector3d p_cam = T_cam_imu * p_imu;

    const std::optional<Eigen::Vector2d> projected =
        Project(camera_params, p_cam);
    if (!projected) continue;

    const Eigen::Vector2d pixel = (*projected) * scale;
    if (!InFrame(image, pixel, WarpedMargin(selected.warp, half_patch))) {
      continue;
    }

    matches.push_back(CreatePhotometricMatch(
        image, pixel, p_imu, *selected.reference_patch, pyramid_level,
        update_params.patch_size, selected.warp));
  }

  matches.shrink_to_fit();
  return matches;
}

IekfSystem BuildCameraSystem(const std::vector<PhotometricMatch>& matches,
                             const State& state,
                             const CameraParams& camera_params,
                             double intensity_noise_std,
                             bool estimate_exposure) {
  IekfSystem system;

  const double weight = 1.0 / (intensity_noise_std * intensity_noise_std);
  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();
  const Eigen::Matrix3d R_cam_imu = T_cam_imu.rotationMatrix();

  for (const auto& match : matches) {
    const Eigen::Vector3d& p_imu = match.point_imu;
    const Eigen::Vector3d p_cam = T_cam_imu * match.point_imu;

    // dU_dP is ∂u/∂p_cam. Move the 3D point in the camera frame, how far
    // the pixel u slides.
    const Eigen::Matrix<double, 2, 3> dU_dp =
        ProjectionJacobian(camera_params, p_cam);
    // dP_dtheta is ∂p_cam/∂δθ. Rotate the body a little, how the point's
    // camera-frame coordinates change.
    const Eigen::Matrix3d dP_dtheta = R_cam_imu * Skew(p_imu);
    // dP_dp is ∂p_cam/∂δp. Translate the body a little, how the point's
    // camera-frame coordinates change.
    const Eigen::Matrix3d dP_dp =
        -R_cam_imu * state.rotation.matrix().transpose();

    // Patches keep their brightness scale, so exposure is observable and
    // scales the image gradient too.
    const double gradient_scale = state.inv_exposure;

    for (Eigen::Index i = 0; i < match.current_patch.size(); ++i) {
      const double residual =
          match.reference_inv_exposure * match.reference_patch[i] -
          state.inv_exposure * match.current_patch[i];
      const Eigen::RowVector3d row =
          gradient_scale * (match.gradients.row(i) * dU_dp);

      ErrorState jacobian = ErrorState::Zero();
      jacobian.segment<3>(kIdxRotation) = -(row * dP_dtheta).transpose();
      jacobian.segment<3>(kIdxPosition) = -(row * dP_dp).transpose();
      if (estimate_exposure) {
        jacobian(kIdxInvExposure) = -match.current_patch[i];
      }

      system.hth += weight * jacobian * jacobian.transpose();
      system.htz += weight * jacobian * -residual;
    }
  }

  return system;
}

}  // namespace livo_slam