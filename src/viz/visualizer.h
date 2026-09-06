#pragma once

#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "core/config/camera_params.h"
#include "core/config/viz_params.h"
#include "core/types/cloud.h"
#include "core/types/plane.h"
#include "core/types/state.h"
#include "core/types/visual_observation.h"

namespace livo_slam {

// Built without LIVO_SLAM_ENABLE_RERUN, every method is a no-op and Rerun is
// not linked, so callers need no #ifdefs.
class Visualizer {
 public:
  Visualizer(const VizParams& params, const CameraParams& camera_params,
             std::string app_id = "livo_slam");
  ~Visualizer();

  Visualizer(const Visualizer&) = delete;
  Visualizer& operator=(const Visualizer&) = delete;
  Visualizer(Visualizer&&) = delete;
  Visualizer& operator=(Visualizer&&) = delete;

  void SetTime(std::int64_t group_index, double sensor_time);

  void LogState(const State& state);
  void LogCloud(const Cloud& cloud);
  void LogMap(const std::vector<Plane>& planes);

  // The current image, with the visual points that project into it drawn on
  // top and coloured by whether they passed the photometric gate.
  void LogCamera(const cv::Mat& image,
                 const std::vector<VisualObservation>& observations);

  // The visual points in 3D.
  void LogVisualPoints(const std::vector<VisualObservation>& observations);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_slam
