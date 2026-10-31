#ifndef ICD_RASTER_PATH_H
#define ICD_RASTER_PATH_H
#include <vector>
#include "control_fsm.h"
namespace icd {
struct RasterPoint { int pan, tilt; };
// 4096 ticks/revolution. Round DOWN so targets never exceed +/-1 degree.
enum { RASTER_HALF_SPAN=4096/360, RASTER_ROW_STEP=2, RASTER_TOLERANCE=1 };
inline std::vector<RasterPoint> rasterPath(int pan,int tilt) {
    if(pan<ControlFsm::PAN_MIN || pan>ControlFsm::PAN_MAX ||
       tilt<ControlFsm::TILT_MIN || tilt>ControlFsm::TILT_MAX) return {};
    // Intersect the requested square with travel limits; do not shift it inward.
    const int left=std::max(int(ControlFsm::PAN_MIN),pan-RASTER_HALF_SPAN);
    const int right=std::min(int(ControlFsm::PAN_MAX),pan+RASTER_HALF_SPAN);
    const int bottom=std::max(int(ControlFsm::TILT_MIN),tilt-RASTER_HALF_SPAN);
    const int top=std::min(int(ControlFsm::TILT_MAX),tilt+RASTER_HALF_SPAN);
    std::vector<RasterPoint> points;
    bool leftToRight=true;
    for(int row=top;;row=std::max(bottom,row-RASTER_ROW_STEP)) {
        points.push_back(RasterPoint{leftToRight?left:right,row});
        points.push_back(RasterPoint{leftToRight?right:left,row});
        if(row==bottom) break;
        leftToRight=!leftToRight;
    }
    return points;
}
}
#endif
