#pragma once
#include "capture.hpp"
#include <functional>
#include <memory>
#include <string>

namespace device {
// The common turret runtime owns the UART, feedback and ICD state machine.
// OBS supplies capture; EFF has no dependency on a concrete LiDAR backend.
using CaptureFactory = std::function<std::unique_ptr<lidar::Capture>(bool, const lidar::Config&)>;
int run_device(int argc, char** argv, const std::string& role, CaptureFactory capture = {});
}
