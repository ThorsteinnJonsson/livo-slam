#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace livo_slam {

struct ImageFrame {
  double timestamp = 0.0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string encoding;
  std::vector<std::uint8_t> data;
};

}  // namespace livo_slam
