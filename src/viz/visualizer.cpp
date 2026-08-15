#include "viz/visualizer.h"

#ifdef LIVO_SLAM_ENABLE_RERUN

#include <algorithm>
#include <print>
#include <rerun.hpp>
#include <string_view>
#include <vector>

namespace livo_slam {

struct Visualizer::Impl {
  rerun::RecordingStream rec;
  std::vector<rerun::Position3D> trajectory;

  explicit Impl(const VizParams& params, const std::string& app_id)
      : rec(app_id) {
    if (const rerun::Error err = rec.connect_grpc(params.grpc_url);
        err.is_err()) {
      std::println("rerun: connect to {} failed ({})", params.grpc_url,
                   err.description);
      return;
    }
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);
    std::println("rerun: streaming to {}", params.grpc_url);
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
    rec.log(entity,
            rerun::Points3D(positions).with_colors(color).with_radii(0.05F));
  }
};

// Blue at kMapColorZMin through to red at kMapColorZMax. Fixed so the
// gradient does not rescale as the map grows.
rerun::Color HeightColor(double z) {
  constexpr double kMapColorZMin = -2.0;
  constexpr double kMapColorZMax = 20.0;
  const double t =
      std::clamp((z - kMapColorZMin) / (kMapColorZMax - kMapColorZMin), 0.0,
                 1.0);
  return rerun::Color(static_cast<std::uint8_t>(255.0 * t), 60,
                      static_cast<std::uint8_t>(255.0 * (1.0 - t)));
}

Visualizer::Visualizer(const VizParams& params, std::string app_id)
    : impl_(params.enabled ? std::make_unique<Impl>(params, app_id) : nullptr) {
}

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

  const Eigen::Vector3d& p = state.position;
  const Eigen::Matrix3d r = state.rotation.matrix();

  impl_->rec.log(
      "world/imu",
      rerun::Transform3D::from_translation_mat3x3(
          rerun::Vec3D{static_cast<float>(p.x()), static_cast<float>(p.y()),
                       static_cast<float>(p.z())},
          rerun::Mat3x3{
              {static_cast<float>(r(0, 0)), static_cast<float>(r(1, 0)),
               static_cast<float>(r(2, 0)), static_cast<float>(r(0, 1)),
               static_cast<float>(r(1, 1)), static_cast<float>(r(2, 1)),
               static_cast<float>(r(0, 2)), static_cast<float>(r(1, 2)),
               static_cast<float>(r(2, 2))}}),
      rerun::TransformAxes3D(0.5F));

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
  impl_->LogPoints("world/cloud", cloud, rerun::Color(200, 40, 40));
}

void Visualizer::LogDeskewedCloud(const Cloud& cloud) {
  if (!impl_) {
    return;
  }
  impl_->LogPoints("world/cloud_deskewed", cloud, rerun::Color(40, 200, 200));
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

  impl_->rec.log("world/map", rerun::Points3D(positions)
                                  .with_colors(colors)
                                  .with_radii(0.05F));
}

}  // namespace livo_slam

#else

namespace livo_slam {

struct Visualizer::Impl {};

Visualizer::Visualizer(const VizParams&, std::string) : impl_(nullptr) {}
Visualizer::~Visualizer() = default;

void Visualizer::SetTime(std::int64_t, double) {}
void Visualizer::LogState(const State&) {}
void Visualizer::LogCloud(const Cloud&) {}
void Visualizer::LogDeskewedCloud(const Cloud&) {}
void Visualizer::LogMap(const std::vector<Plane>&) {}

}  // namespace livo_slam

#endif
