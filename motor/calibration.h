#ifndef ANGLE_CALIBRATION_H
#define ANGLE_CALIBRATION_H
#include "control_fsm.h"
#include <cstddef>

namespace calibration {
struct Coordinates { double pan, tilt; };
struct Target { int pan, tilt; };

class Map {
    const Point* points;
    std::size_t count;
    static double panNode(const Point& point) {
        return point.azimuthDeg*4096.0/360.0+point.panTicks;
    }
public:
    explicit Map(const Point* table=POINTS, std::size_t size=sizeof(POINTS)/sizeof(POINTS[0]))
        : points(table), count(size) {}

    bool valid() const {
        if(!points || count<2) return false;
        for(std::size_t i=0;i<count;++i) {
            const auto& point=points[i];
            if(!std::isfinite(point.azimuthDeg) || !std::isfinite(point.panTicks) ||
               !std::isfinite(point.tiltTicks) || point.azimuthDeg<0 || point.azimuthDeg>360 ||
               std::abs(point.panTicks)>4096 || std::abs(point.tiltTicks)>4096) return false;
            // A monotonic pan mapping allows measured ticks to be converted back.
            if(i>0 && (point.azimuthDeg<=points[i-1].azimuthDeg ||
                       panNode(point)<=panNode(points[i-1]))) return false;
        }
        return true;
    }

    Coordinates extraTicks(double azimuthDeg) const {
        if(azimuthDeg<=points[0].azimuthDeg) return {points[0].panTicks,points[0].tiltTicks};
        for(std::size_t i=1;i<count;++i) {
            if(azimuthDeg<=points[i].azimuthDeg) {
                const auto& low=points[i-1];
                const auto& high=points[i];
                double fraction=(azimuthDeg-low.azimuthDeg)/(high.azimuthDeg-low.azimuthDeg);
                return {low.panTicks+fraction*(high.panTicks-low.panTicks),
                        low.tiltTicks+fraction*(high.tiltTicks-low.tiltTicks)};
            }
        }
        return {points[count-1].panTicks,points[count-1].tiltTicks};
    }

    Target apply(double nominalPan,double nominalTilt) const {
        auto extra=extraTicks(nominalPan*360.0/4096.0);
        return {
            ControlFsm::clampPan(static_cast<int>(std::lround(nominalPan+ControlFsm::PAN_OFFSET_TICKS+extra.pan))),
            ControlFsm::clampTilt(static_cast<int>(std::lround(nominalTilt+ControlFsm::TILT_OFFSET_TICKS+extra.tilt)))
        };
    }

    Coordinates remove(int pan,int tilt) const {
        double measuredPan=pan-ControlFsm::PAN_OFFSET_TICKS;
        if(!valid()) return {measuredPan,double(tilt-ControlFsm::TILT_OFFSET_TICKS)};
        double azimuthDeg;
        if(measuredPan<=panNode(points[0])) {
            azimuthDeg=(measuredPan-points[0].panTicks)*360.0/4096.0;
        } else if(measuredPan>=panNode(points[count-1])) {
            azimuthDeg=(measuredPan-points[count-1].panTicks)*360.0/4096.0;
        } else {
            std::size_t i=1;
            while(measuredPan>panNode(points[i])) ++i;
            double fraction=(measuredPan-panNode(points[i-1]))/(panNode(points[i])-panNode(points[i-1]));
            azimuthDeg=points[i-1].azimuthDeg+fraction*(points[i].azimuthDeg-points[i-1].azimuthDeg);
        }
        auto extra=extraTicks(azimuthDeg);
        return {azimuthDeg*4096.0/360.0,tilt-ControlFsm::TILT_OFFSET_TICKS-extra.tilt};
    }
};
}
#endif
