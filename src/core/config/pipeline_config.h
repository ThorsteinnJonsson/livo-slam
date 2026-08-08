#pragma once

#include "core/config/init_params.h"
#include "core/config/initial_state_params.h"

namespace livo_slam {

struct PipelineConfig {
  ImuInitParams init;
  InitialStateParams initial;
};

}  // namespace livo_slam
