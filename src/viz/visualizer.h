#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "core/config/viz_params.h"
#include <vector>

#include "core/types/cloud.h"
#include "core/types/plane.h"
#include "core/types/state.h"

namespace livo_slam {

// Built without LIVO_SLAM_ENABLE_RERUN, every method is a no-op and Rerun is
// not linked, so callers need no #ifdefs.
class Visualizer {
 public:
  explicit Visualizer(const VizParams& params,
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

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace livo_slam
