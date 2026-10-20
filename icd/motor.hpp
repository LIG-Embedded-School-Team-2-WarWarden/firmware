#pragma once
#include "client.hpp"
#include <algorithm>
#include <cmath>
#ifndef _WIN32
#include "motor.h"
#endif
namespace device {
struct Calibration {
    double pan_zero=2048,tilt_zero=2048,ticks_per_turn=4096,pan_sign=1,tilt_sign=1;
    int pan_min=1024,pan_max=3072,tilt_min=1536,tilt_max=2560;
    std::string id;
    double degrees(double ticks,bool pan)const{return (ticks-(pan?pan_zero:tilt_zero))*360/ticks_per_turn*(pan?pan_sign:tilt_sign);}
    int ticks(double deg,bool pan)const{return static_cast<int>(std::lround((pan?pan_zero:tilt_zero)+deg*ticks_per_turn/360/(pan?pan_sign:tilt_sign)));}
};
struct Sample { bool pan_valid=false,tilt_valid=false;double pan=0,tilt=0,pan_at=0,tilt_at=0;std::uint64_t sequence=0; };
class Motor {
    bool simulation_;Calibration calibration_;Sample sample_;Mono::time_point read_{};
    double simulated_pan_=0,simulated_tilt_=0;
public:
    Motor(bool simulation,Calibration calibration,const std::string& uart,unsigned baud):simulation_(simulation),calibration_(std::move(calibration)){
        if(!std::isfinite(calibration_.ticks_per_turn) || !std::isfinite(calibration_.pan_zero) || !std::isfinite(calibration_.tilt_zero) || calibration_.ticks_per_turn<=0 || std::abs(calibration_.pan_sign)!=1 || std::abs(calibration_.tilt_sign)!=1 || calibration_.pan_min>=calibration_.pan_max || calibration_.tilt_min>=calibration_.tilt_max || calibration_.pan_min<0 || calibration_.tilt_min<0 || calibration_.pan_max>65535 || calibration_.tilt_max>65535 || calibration_.id.empty())throw std::invalid_argument("verified axis calibration and limits required");
        if(!simulation_){
#ifdef _WIN32
            throw std::runtime_error("real UART motor backend requires Linux");
#else
            if(!motorInit(uart.c_str(),baud))throw std::runtime_error("motor UART open/configuration failed");
#endif
        }
    }
    ~Motor(){
#ifndef _WIN32
        if(!simulation_)motorClose();
#endif
    }
    const Calibration& calibration()const{return calibration_;}
    const Sample& sample()const{return sample_;}
    bool poll(bool force=false){
        if(!force&&Mono::now()-read_<std::chrono::milliseconds(50))return false;
        read_=Mono::now();++sample_.sequence;
        if(simulation_){sample_.pan_valid=sample_.tilt_valid=true;sample_.pan=simulated_pan_;sample_.tilt=simulated_tilt_;sample_.pan_at=sample_.tilt_at=wall();}
#ifndef _WIN32
        else{const auto p=motorReadPositions();sample_.pan_valid=p.panValid;sample_.tilt_valid=p.tiltValid;
            if(p.panValid){sample_.pan=calibration_.degrees(p.panTicks,true);sample_.pan_at=p.panUtcMs/1000.;}
            if(p.tiltValid){sample_.tilt=calibration_.degrees(p.tiltTicks,false);sample_.tilt_at=p.tiltUtcMs/1000.;}}
#endif
        return true;
    }
    bool in_range(double pan,double tilt)const{
        if(!std::isfinite(pan)||!std::isfinite(tilt))return false;
        const auto& a=calibration_;
        return pan>=std::min(a.degrees(a.pan_min,true),a.degrees(a.pan_max,true))&&pan<=std::max(a.degrees(a.pan_min,true),a.degrees(a.pan_max,true))&&tilt>=std::min(a.degrees(a.tilt_min,false),a.degrees(a.tilt_max,false))&&tilt<=std::max(a.degrees(a.tilt_min,false),a.degrees(a.tilt_max,false));
    }
    bool move(double pan,double tilt){
        if(!in_range(pan,tilt))return false;
        const auto p=calibration_.ticks(pan,true),t=calibration_.ticks(tilt,false);
        if(p<calibration_.pan_min || p>calibration_.pan_max || t<calibration_.tilt_min || t>calibration_.tilt_max)return false;
        if(simulation_){simulated_pan_=pan;simulated_tilt_=tilt;return true;}
#ifndef _WIN32
        return motorMove(p,t,200,200);
#else
        return false;
#endif
    }
    bool hold(){poll(true);return sample_.pan_valid && sample_.tilt_valid && wall()-sample_.pan_at<=.25 && wall()-sample_.tilt_at<=.25 && move(sample_.pan,sample_.tilt);}
};
inline void fill_angles(pb::AngleSample* a,const Sample& sample){
    a->set_sample_sequence(sample.sequence);a->set_pan_valid(sample.pan_valid);a->set_tilt_valid(sample.tilt_valid);
    if(sample.pan_valid){a->set_pan_deg(sample.pan);a->set_pan_measured_at(utc(sample.pan_at));}
    if(sample.tilt_valid){a->set_tilt_deg(sample.tilt);a->set_tilt_measured_at(utc(sample.tilt_at));}
    if(!sample.pan_valid||!sample.tilt_valid)a->set_angle_error_code(pb::REASON_CODE_ANGLE_READ_FAILED);
}
}
