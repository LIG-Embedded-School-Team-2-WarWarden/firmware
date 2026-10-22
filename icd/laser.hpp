#pragma once
#include "client.hpp"
#include <optional>
#ifndef _WIN32
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#endif

namespace device {
// Output-enable line feedback; this does not measure optical emission.
// Real hardware is opt-in. No GPIO is opened for OBS or an unconfigured EFF.
class LaserOutput {
    bool simulation_, enabled_, simulated_on_ = false;
    int line_ = -1, estop_line_ = -1;
    bool estop_latched_ = false;
public:
    LaserOutput(bool simulation, bool effector, const std::string &chip, unsigned offset, unsigned estop_offset)
        : simulation_(simulation), enabled_(effector && (simulation || !chip.empty())) {
        if (!enabled_ || simulation_) return;
#ifdef _WIN32
        throw std::invalid_argument("real laser GPIO requires Linux");
#else
        const int fd = open(chip.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0) throw std::runtime_error("cannot open laser GPIO chip");
        gpiohandle_request request{};
        request.lineoffsets[0] = offset;
        request.flags = GPIOHANDLE_REQUEST_OUTPUT;
        request.lines = 1;
        std::strcpy(request.consumer_label, "mfs-eff-laser");
        const int ok = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &request);
        if (ok < 0) { close(fd); throw std::runtime_error("cannot acquire laser GPIO line OFF"); }
        line_ = request.fd;
        gpiohandle_request stop{};
        stop.lineoffsets[0] = estop_offset; stop.flags = GPIOHANDLE_REQUEST_INPUT; stop.lines = 1;
        std::strcpy(stop.consumer_label, "mfs-eff-estop");
        const int stop_ok = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &stop);
        close(fd);
        if (stop_ok < 0) {
            close(line_); line_ = -1;
            throw std::runtime_error("cannot acquire active-high emergency stop input");
        }
        estop_line_ = stop.fd;
        if (read() != pb::LASER_STATE_OFF) {
            close(line_); close(estop_line_); line_ = estop_line_ = -1;
            throw std::runtime_error("laser initial OFF unverified");
        }
#endif
    }
    ~LaserOutput() {
        if (enabled_) set(false);
#ifndef _WIN32
        if (line_ >= 0) close(line_);
        if (estop_line_ >= 0) close(estop_line_);
#endif
    }
    bool supported() const { return enabled_; }
    void latch_stop() { estop_latched_ = true; set(false); }
    bool estop_latched() {
        if (estop_latched_) return true;
#ifndef _WIN32
        if (enabled_ && !simulation_) {
            gpiohandle_data data{};
            if (estop_line_ < 0 || ioctl(estop_line_, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) != 0 || data.values[0])
                latch_stop();
        }
#endif
        return estop_latched_;
    }
    pb::LaserState read() const {
        if (!enabled_) return pb::LASER_STATE_UNKNOWN;
        if (simulation_) return simulated_on_ ? pb::LASER_STATE_ON : pb::LASER_STATE_OFF;
#ifndef _WIN32
        gpiohandle_data data{};
        if (line_ >= 0 && ioctl(line_, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) == 0)
            return data.values[0] ? pb::LASER_STATE_ON : pb::LASER_STATE_OFF;
#endif
        return pb::LASER_STATE_UNKNOWN;
    }
    bool set(bool on) {
        if (!enabled_) return false;
        if (on && estop_latched()) return false;
        if (simulation_) simulated_on_ = on;
        else {
#ifndef _WIN32
            gpiohandle_data data{}; data.values[0] = on ? 1 : 0;
            if (line_ < 0 || ioctl(line_, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) != 0) return false;
#else
            return false;
#endif
        }
        return read() == (on ? pb::LASER_STATE_ON : pb::LASER_STATE_OFF);
    }
};
}
