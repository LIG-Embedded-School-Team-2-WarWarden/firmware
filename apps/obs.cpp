#include "runtime.hpp"
#include <stdexcept>

int main(int argc, char** argv) {
    return device::run_device(argc, argv, "obs", [](bool simulation, const lidar::Config& config) {
        if (simulation) return lidar::simulated_capture();
#ifdef LIDAR_WITH_XTSDK
        return lidar::xt_capture(config);
#else
        throw std::runtime_error("real OBS requires build with LIDAR_WITH_XTSDK=ON");
#endif
    });
}
