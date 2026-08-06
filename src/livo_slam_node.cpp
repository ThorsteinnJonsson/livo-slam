#include "livo_slam_node.h"

namespace livo_slam {

LivoSlamNode::LivoSlamNode() : Node("livo_slam") {
  RCLCPP_INFO(get_logger(), "livo_slam node started");
}

}  // namespace livo_slam
