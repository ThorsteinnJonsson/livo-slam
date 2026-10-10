#pragma once

#include <string>

namespace livo_slam {

struct VizParams {
  bool enabled = true;
  bool color_cloud = false;
  double color_cloud_voxel_size = 0.02;
  double color_cloud_max_range = 20.0;
  // Zero disables it. Rejects samples on strong edges, which also throws away
  // the fine detail, so it is off unless something needs it.
  double color_cloud_max_contrast = 0.0;
  std::string grpc_url = "rerun+http://127.0.0.1:9876/proxy";
};

}  // namespace livo_slam
