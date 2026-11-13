#ifndef CROSS_RASTER_PATH_H
#define CROSS_RASTER_PATH_H
#include "control_fsm.h"
#include <vector>
namespace scan {
struct Point { int pan, tilt; };
// Timed endpoints: pan reverses on every command. Two traversals per height.
// The first traversal of a new row also changes tilt (a shallow diagonal).
inline std::vector<Point> path(int p, int t, int span, int /*step*/) {
    if (span < 1) return {};

    // 중심에서 좌우로 얼마나 이동할지 이 값만 바꾸면 돼.
    constexpr double SIDE_OFFSET_DEG = 1.0;
    const int sideTicks = static_cast<int>(
        std::lround(SIDE_OFFSET_DEG * 4096.0 / 360.0));

    const int center = ControlFsm::clampPan(p);
    const int left   = ControlFsm::clampPan(center - sideTicks);
    const int right  = ControlFsm::clampPan(center + sideTicks);
    const int top    = ControlFsm::clampTilt(t + span);
    const int bottom = ControlFsm::clampTilt(t - span);

    std::vector<Point> out{{center, top}, {center, bottom}};

    // 아래쪽 높이를 유지하면서 다음 열로 옮긴 뒤 위→아래로 훑어.
    auto sweepColumn = [&](int pan) {
        if (out.back().pan == pan) return;
        out.push_back({pan, bottom});
        out.push_back({pan, top});
        out.push_back({pan, bottom});
    };

    sweepColumn(left);
    sweepColumn(right);
    return out;
}
}
#endif
