#pragma once

#include <string>

namespace livo_slam {

struct VizParams {
  bool enabled = true;
  bool color_cloud = false;
  double color_cloud_voxel_size = 0.05;
  std::string grpc_url = "rerun+http://127.0.0.1:9876/proxy";
};

}  // namespace livo_slam
