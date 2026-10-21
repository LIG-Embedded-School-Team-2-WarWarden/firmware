#pragma once
#include "capture.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>

namespace device {
// Operator evidence describes the sensor timestamp and full exposure interval.
// Neither callback arrival nor command completion substitutes for acquisition time.
struct AcquisitionProfile {
    std::string id,evidence,board_boot_id,sensor_ip,sensor_profile;
    double offset_seconds=0,uncertainty_ms=0,start_offset_ms=0,end_offset_ms=0;
    double valid_from=0,valid_until=0,max_pose_gap_ms=200;
    std::uint32_t sync_type=0,sync_state=0,frame_protocol_version=0;
};
inline bool map_acquisition(lidar::Frame& f,const AcquisitionProfile& p,double now) {
    if(p.id.empty()||p.evidence.empty()||!std::isfinite(p.offset_seconds)||!std::isfinite(p.uncertainty_ms)||
       p.uncertainty_ms<1||p.uncertainty_ms>100||!std::isfinite(p.start_offset_ms)||!std::isfinite(p.end_offset_ms)||
       p.start_offset_ms>0||p.end_offset_ms<0||p.start_offset_ms>=p.end_offset_ms||
       !std::isfinite(p.valid_from)||!std::isfinite(p.valid_until)||p.valid_from>now||now>p.valid_until||
       !std::isfinite(p.max_pose_gap_ms)||p.max_pose_gap_ms<=0||p.max_pose_gap_ms>250||
       f.sdk_nanoseconds>=1000000000||f.sync_type!=p.sync_type||f.sync_state!=p.sync_state||f.protocol_version!=p.frame_protocol_version)return false;
    const double at=std::round((double(f.sdk_seconds)+f.sdk_nanoseconds/1e9+p.offset_seconds)*1000)/1000;
    const double start=at+p.start_offset_ms/1000,end=at+p.end_offset_ms/1000;
    if(!std::isfinite(at)||start<p.valid_from||end>p.valid_until||end>now||now-start>1)return false;
    f.captured_at=at;f.captured_start=start;f.captured_end=end;f.uncertainty_ms=p.uncertainty_ms;f.mapping_id=p.id;
    return true;
}
struct PoseReading {
    double observed_at=0,pan_at=0,tilt_at=0,pan=0,tilt=0;
    bool valid=false,stopped=false;
    std::uint64_t generation=0;
};
enum class PoseQuality { Invalid,Pending,Verified };
struct CapturedPose { PoseQuality quality=PoseQuality::Invalid;double pan=0,tilt=0; };
class PoseHistory {
    std::deque<PoseReading> samples_;
public:
    void clear(){samples_.clear();}
    void add(PoseReading s){
        if(!samples_.empty() && (s.observed_at<samples_.back().observed_at||s.observed_at-samples_.back().observed_at>.5))clear();
        samples_.push_back(s);
        while(samples_.size()>256||(!samples_.empty()&&s.observed_at-samples_.front().observed_at>3))samples_.pop_front();
    }
    CapturedPose qualify(const lidar::Frame& f,double now,double gap_ms,double tolerance,std::uint64_t generation)const {
        CapturedPose out;
        if(!f.captured_start||!f.captured_at||!f.captured_end||!f.uncertainty_ms||samples_.empty())return out;
        const double start=*f.captured_start-*f.uncertainty_ms/1000,end=*f.captured_end+*f.uncertainty_ms/1000;
        if(!std::isfinite(start)||!std::isfinite(end)||start>end||*f.captured_start>*f.captured_at||*f.captured_at>*f.captured_end||now-start>1)return out;
        if(end>samples_.back().pan_at||end>samples_.back().tilt_at){
            if(now-end<gap_ms/1000)out.quality=PoseQuality::Pending;
            return out;
        }
        double values[2]{};
        for(int axis=0;axis<2;++axis){
            auto at=[&](const PoseReading& s){return axis?s.tilt_at:s.pan_at;};
            auto angle=[&](const PoseReading& s){return axis?s.tilt:s.pan;};
            std::size_t left=samples_.size(),right=samples_.size();
            for(std::size_t i=0;i<samples_.size();++i){
                if(at(samples_[i])<=start)left=i;
                if(at(samples_[i])>=end){right=i;break;}
            }
            if(left==samples_.size()||right==samples_.size()||left>right)return out;
            const double anchor=angle(samples_[left]);double distance=1e100;
            for(std::size_t i=left;i<=right;++i){const auto& s=samples_[i];
                if(!s.valid||!s.stopped||s.generation!=generation||!std::isfinite(at(s))||!std::isfinite(angle(s))||
                   s.observed_at-at(s)<0||s.observed_at-at(s)>.25||std::abs(angle(s)-anchor)>tolerance)return out;
                if(i>left&&(at(s)<=at(samples_[i-1])||at(s)-at(samples_[i-1])>gap_ms/1000))return out;
                const auto d=std::abs(at(s)-*f.captured_at);if(d<distance){distance=d;values[axis]=angle(s);}
            }
        }
        out.quality=PoseQuality::Verified;out.pan=values[0];out.tilt=values[1];return out;
    }
};
}
