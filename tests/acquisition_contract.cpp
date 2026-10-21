#include "acquisition.hpp"
#include <iostream>
#include <stdexcept>
using namespace device;
static void require(bool ok,const char* name){if(!ok)throw std::runtime_error(name);}
static AcquisitionProfile profile(){AcquisitionProfile p;p.id="measured-clock";p.evidence="fixture evidence";p.valid_from=90;p.valid_until=110;p.uncertainty_ms=1;p.start_offset_ms=-10;p.end_offset_ms=10;p.sync_type=2;p.sync_state=1;p.frame_protocol_version=3;return p;}
static lidar::Frame frame(){lidar::Frame f;f.sdk_seconds=100;f.sdk_nanoseconds=50000000;f.sync_type=2;f.sync_state=1;f.protocol_version=3;return f;}
static PoseReading reading(double t,bool stopped=true,std::uint64_t generation=7){return {t+.001,t,t,0,0,true,stopped,generation};}
int main(){try{
    auto p=profile();auto f=frame();require(map_acquisition(f,p,100.1),"verified mapping");
    require(f.mapping_id==p.id&&std::abs(*f.captured_at-100.05)<1e-8,"mapped SDK time");
    require(f.sdk_seconds==100&&f.sdk_nanoseconds==50000000,"raw time preserved");
    auto bad=frame();bad.sync_state=0;require(!map_acquisition(bad,p,100.1)&&!bad.captured_at,"lost sync cannot qualify");
    bad=frame();require(!map_acquisition(bad,p,111),"expired evidence");
    bad=frame();require(!map_acquisition(bad,p,100.04),"future exposure");
    auto wrong=p;wrong.evidence.clear();require(!map_acquisition(bad,wrong,100.1),"evidence required");
    wrong=p;wrong.start_offset_ms=10;require(!map_acquisition(bad,wrong,100.1),"exposure interval required");
    PoseHistory h;h.add(reading(100.02));h.add(reading(100.05));
    require(h.qualify(f,100.055,200,.5,7).quality==PoseQuality::Pending,"wait for measured post exposure sample");
    h.add(reading(100.08));require(h.qualify(f,100.1,200,.5,7).quality==PoseQuality::Verified,"bracketed stationary pose");
    require(h.qualify(f,100.1,20,.5,7).quality==PoseQuality::Invalid,"sample gap");
    require(h.qualify(f,100.1,200,.5,8).quality==PoseQuality::Invalid,"move generation boundary");
    PoseHistory moving;moving.add(reading(100.02));moving.add(reading(100.05,false));moving.add(reading(100.08));
    require(moving.qualify(f,100.1,200,.5,7).quality==PoseQuality::Invalid,"motion in exposure");
    PoseHistory invalid;invalid.add(reading(100.02));auto sample=reading(100.05);sample.valid=false;invalid.add(sample);invalid.add(reading(100.08));
    require(invalid.qualify(f,100.1,200,.5,7).quality==PoseQuality::Invalid,"failed UART read");
    PoseHistory absent;absent.add(reading(100.05));absent.add(reading(100.08));
    require(absent.qualify(f,100.1,200,.5,7).quality==PoseQuality::Invalid,"no extrapolation before history");
    PoseHistory drifting;drifting.add(reading(100.02));sample=reading(100.05);sample.pan=.4;drifting.add(sample);sample=reading(100.08);sample.pan=.8;drifting.add(sample);
    require(drifting.qualify(f,100.1,200,.5,7).quality==PoseQuality::Invalid,"accumulated drift");
    h.clear();require(h.qualify(f,100.1,200,.5,7).quality==PoseQuality::Invalid,"clock/session reset");
    std::cout<<"Acquisition mapping and bracketed stationary UART pose contract passed.\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
