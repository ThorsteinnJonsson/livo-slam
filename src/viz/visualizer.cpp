#include "viz/visualizer.h"

#ifdef LIVO_SLAM_ENABLE_RERUN

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <print>
#include <rerun.hpp>
#include <string_view>
#include <vector>

namespace livo_slam {

struct Visualizer::Impl {
  rerun::RecordingStream rec;
  std::vector<rerun::Position3D> trajectory;
  Sophus::SE3d T_imu_cam;

  Impl(const VizParams& params, const CameraParams& camera_params,
       const std::string& app_id)
      : rec(app_id), T_imu_cam(camera_params.T_imu_cam) {
    if (const rerun::Error err = rec.connect_grpc(params.grpc_url);
        err.is_err()) {
      std::println("rerun: connect to {} failed ({})", params.grpc_url,
                   err.description);
      return;
    }
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    LogPinhole(camera_params);
    std::println("rerun: streaming to {}", params.grpc_url);
  }

  void LogPinhole(const CameraParams& camera_params) {
    const auto f = [](double v) { return static_cast<float>(v); };
    rec.log_static(
        "world/camera",
        rerun::Pinhole(
            rerun::components::PinholeProjection(rerun::Mat3x3{
                {f(camera_params.fx), 0.0F, 0.0F, 0.0F, f(camera_params.fy),
                 0.0F, f(camera_params.cx), f(camera_params.cy), 1.0F}}))
            .with_resolution(static_cast<int>(camera_params.width),
                             static_cast<int>(camera_params.height)));
  }

  void LogPoints(std::string_view entity, const Cloud& cloud,
                 rerun::Color color) {
    std::vector<rerun::Position3D> positions;
    positions.reserve(cloud.points.size());
    for (const Point& point : cloud.points) {
      positions.emplace_back(static_cast<float>(point.position.x()),
                             static_cast<float>(point.position.y()),
                             static_cast<float>(point.position.z()));
    }

    rec.log_static(
        entity,
        rerun::Points3D(positions).with_colors(color).with_radii(0.075F));
  }
};

// Sweeps hue from blue (low) through green to red (high), saturation and
// value pinned to 1.
rerun::Color HueColor(float t) {
  const float h = (1.0F - std::clamp(t, 0.0F, 1.0F)) * 4.0F;
  const float x = 1.0F - std::abs(std::fmod(h, 2.0F) - 1.0F);
  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;
  if (h < 1.0F) {
    r = 1.0F, g = x;
  } else if (h < 2.0F) {
    r = x, g = 1.0F;
  } else if (h < 3.0F) {
    g = 1.0F, b = x;
  } else if (h < 4.0F) {
    g = x, b = 1.0F;
  } else if (h < 5.0F) {
    r = x, b = 1.0F;
  } else {
    r = 1.0F, b = x;
  }
  const auto to8 = [](float c) {
    return static_cast<std::uint8_t>(c * 255.0F);
  };
  return rerun::Color(to8(r), to8(g), to8(b));
}

// Fixed range so the gradient does not rescale as the map grows.
rerun::Color HeightColor(double z) {
  constexpr double kMapColorZMin = -5.0;
  constexpr double kMapColorZMax = 20.0;
  return HueColor(static_cast<float>((z - kMapColorZMin) /
                                     (kMapColorZMax - kMapColorZMin)));
}

Visualizer::Visualizer(const VizParams& params,
                       const CameraParams& camera_params, std::string app_id)
    : impl_(params.enabled
                ? std::make_unique<Impl>(params, camera_params, app_id)
                : nullptr) {}

Visualizer::~Visualizer() = default;

void Visualizer::SetTime(std::int64_t group_index, double sensor_time) {
  if (!impl_) {
    return;
  }
  impl_->rec.set_time_sequence("group", group_index);
  impl_->rec.set_time_duration_secs("sensor_time", sensor_time);
}

void Visualizer::LogState(const State& state) {
  if (!impl_) {
    return;
  }

  const auto to_transform = [](const Sophus::SE3d& pose) {
    const Eigen::Vector3d& t = pose.translation();
    const Eigen::Matrix3d m = pose.rotationMatrix();
    return rerun::Transform3D::from_translation_mat3x3(
        rerun::Vec3D{static_cast<float>(t.x()), static_cast<float>(t.y()),
                     static_cast<float>(t.z())},
        rerun::Mat3x3{{static_cast<float>(m(0, 0)), static_cast<float>(m(1, 0)),
                       static_cast<float>(m(2, 0)), static_cast<float>(m(0, 1)),
                       static_cast<float>(m(1, 1)), static_cast<float>(m(2, 1)),
                       static_cast<float>(m(0, 2)), static_cast<float>(m(1, 2)),
                       static_cast<float>(m(2, 2))}});
  };

  const Sophus::SE3d T_world_imu(state.rotation, state.position);
  impl_->rec.log("world/imu", to_transform(T_world_imu),
                 rerun::TransformAxes3D(0.5F));
  impl_->rec.log("world/camera", to_transform(T_world_imu * impl_->T_imu_cam));

  const Eigen::Vector3d& p = state.position;
  impl_->trajectory.emplace_back(static_cast<float>(p.x()),
                                 static_cast<float>(p.y()),
                                 static_cast<float>(p.z()));
  impl_->rec.log("world/trajectory",
                 rerun::LineStrips3D(rerun::LineStrip3D(impl_->trajectory))
                     .with_colors(rerun::Color(255, 200, 0)));
}

void Visualizer::LogCloud(const Cloud& cloud) {
  if (!impl_) {
    return;
  }
  impl_->LogPoints("world/cloud", cloud, rerun::Color(178, 150, 220));
}

namespace {
rerun::Color MatchColor(bool matched) {
  return matched ? rerun::Color(80, 220, 100) : rerun::Color(220, 70, 70);
}
}  // namespace

void Visualizer::LogCamera(const cv::Mat& image,
                           const std::vector<VisualObservation>& observations) {
  if (!impl_ || image.empty()) {
    return;
  }

  impl_->rec.log_static(
      "world/camera/image",
      rerun::Image(rerun::Collection<std::uint8_t>::borrow(
                       image.data, image.total() * image.elemSize()),
                   rerun::WidthHeight(static_cast<std::uint32_t>(image.cols),
                                      static_cast<std::uint32_t>(image.rows)),
                   rerun::datatypes::ColorModel::L));

  std::vector<rerun::Position2D> pixels;
  std::vector<rerun::Color> colors;
  pixels.reserve(observations.size());
  colors.reserve(observations.size());
  for (const VisualObservation& observation : observations) {
    pixels.emplace_back(static_cast<float>(observation.pixel.x()),
                        static_cast<float>(observation.pixel.y()));
    colors.push_back(MatchColor(observation.matched));
  }

  impl_->rec.log_static(
      "world/camera/image/projections",
      rerun::Points2D(pixels).with_colors(colors).with_radii(2.0F));
}

void Visualizer::LogVisualPoints(
    const std::vector<VisualObservation>& observations) {
  if (!impl_) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;
  positions.reserve(observations.size());
  colors.reserve(observations.size());
  for (const VisualObservation& observation : observations) {
    positions.emplace_back(static_cast<float>(observation.position.x()),
                           static_cast<float>(observation.position.y()),
                           static_cast<float>(observation.position.z()));
    colors.push_back(MatchColor(observation.matched));
  }

  impl_->rec.log_static(
      "world/visual_points",
      rerun::Points3D(positions).with_colors(colors).with_radii(0.15F));
}

void Visualizer::LogColorCloud(std::int64_t batch,
                               const std::vector<ColorPoint>& points) {
  if (!impl_ || points.empty()) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;
  positions.reserve(points.size());
  colors.reserve(points.size());
  for (const ColorPoint& point : points) {
    positions.emplace_back(static_cast<float>(point.position.x()),
                           static_cast<float>(point.position.y()),
                           static_cast<float>(point.position.z()));
    colors.emplace_back(point.r, point.g, point.b);
  }

  impl_->rec.log_static(
      "world/color_cloud/" + std::to_string(batch),
      rerun::Points3D(positions).with_colors(colors).with_radii(0.075F));
}

void Visualizer::LogMap(const std::vector<Plane>& planes) {
  if (!impl_) {
    return;
  }

  std::vector<rerun::Position3D> positions;
  std::vector<rerun::Color> colors;
  positions.reserve(planes.size());
  colors.reserve(planes.size());
  for (const Plane& plane : planes) {
    positions.emplace_back(static_cast<float>(plane.centroid.x()),
                           static_cast<float>(plane.centroid.y()),
                           static_cast<float>(plane.centroid.z()));
    colors.push_back(HeightColor(plane.centroid.z()));
  }

  impl_->rec.log_static(
      "world/map",
      rerun::Points3D(positions).with_colors(colors).with_radii(0.075F));
}

}  // namespace livo_slam

#else

namespace livo_slam {

struct Visualizer::Impl {};

Visualizer::Visualizer(const VizParams&, const CameraParams&, std::string)
    : impl_(nullptr) {}
Visualizer::~Visualizer() = default;

void Visualizer::SetTime(std::int64_t, double) {}
void Visualizer::LogState(const State&) {}
void Visualizer::LogCloud(const Cloud&) {}
void Visualizer::LogMap(const std::vector<Plane>&) {}
void Visualizer::LogCamera(const cv::Mat&,
                           const std::vector<VisualObservation>&) {}
void Visualizer::LogVisualPoints(const std::vector<VisualObservation>&) {}
void Visualizer::LogColorCloud(std::int64_t, const std::vector<ColorPoint>&) {}

}  // namespace livo_slam

#endif
