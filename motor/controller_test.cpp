#include "controller.h"
#include <cassert>
#include <iostream>
#include <sstream>
#include <iomanip>
using namespace icd;
struct Fixture {
    Controller c;
    int moves=0, holds=0, lights=0;
    bool gpioOK=true;
    std::vector<RasterPoint> targets;
    Fixture() {
        c.move=[this](int,int){++moves;};
        c.scanMove=[this](int p,int t){++moves; targets.push_back(RasterPoint{p,t});};
        c.hold=[this](){++holds;};
        c.light=[this](bool){++lights;return gpioOK;};
        c.sample(ControlFsm::panTicks(180),ControlFsm::tiltTicks(30),0);
        c.receive(packet(EF01,0,0),"peer",0);
    }
    std::string send(int id,unsigned seq,std::vector<int> data={},long long now=0) {
        return c.receive(packet(id,seq,0,data),"peer",now);
    }
    void arrive(long long& now) {
        now+=50; c.sample(c.targetPan,c.targetTilt,now); c.tick(now);
        now+=150; c.sample(c.targetPan,c.targetTilt,now); c.tick(now);
    }
};
static void tests() {
    // Measurement margins are independent of commanded limits and arrival tolerance.
    for(int offset=0;offset<=3;++offset) {
        for(int p:{ControlFsm::PAN_MIN-offset,ControlFsm::PAN_MAX+offset})
        for(int t:{ControlFsm::TILT_MIN-offset,ControlFsm::TILT_MAX+offset}) {
            Fixture f; f.c.sample(p,t,0);
            assert(!f.c.fault && f.c.pan==p && f.c.tilt==t);
            assert(word(f.send(EF05,1),6)==ACCEPTED);
            assert(f.c.basePan==ControlFsm::clampPan(p));
            assert(f.c.baseTilt==ControlFsm::clampTilt(t));
            long long now=0;
            size_t count=f.c.scanPath.size();
            for(size_t n=0;n<count+1;++n) {
                assert(f.c.targetPan>=ControlFsm::PAN_MIN && f.c.targetPan<=ControlFsm::PAN_MAX);
                assert(f.c.targetTilt>=ControlFsm::TILT_MIN && f.c.targetTilt<=ControlFsm::TILT_MAX);
                f.arrive(now);
            }
            assert(!f.c.fault && !f.c.demo && !f.c.moving);
        }
    }
    for(auto p:std::vector<RasterPoint>{{ControlFsm::PAN_MIN-4,0},{ControlFsm::PAN_MAX+4,0},
                                      {2048,-4},{2048,ControlFsm::TILT_MAX+4}}) {
        Fixture f; f.c.sample(p.pan,p.tilt,0);
        assert(f.c.fault && f.holds==1);
        assert(word(f.send(EF03,1,{30,180}),6)==DEVICE_ERROR);
        auto message=ControlFsm::rangeError(p.pan,p.tilt);
        assert(message.find("PAN="+std::to_string(p.pan))!=std::string::npos);
        assert(message.find("TILT="+std::to_string(p.tilt))!=std::string::npos);
    }
    {
        Fixture f; f.send(EF03,1,{0,240});
        f.c.sample(ControlFsm::PAN_MAX+3,0,50);
        f.c.sample(ControlFsm::PAN_MAX+3,0,200);
        assert(!f.c.fault && f.c.moving); // 3-tick measurement accepted, NOT arrival
        f.c.sample(ControlFsm::PAN_MAX+2,0,250);
        f.c.sample(ControlFsm::PAN_MAX+2,0,400);
        assert(!f.c.moving);
    }
    // Signed IDs, independent unsigned sequences; wire format remains big-endian.
    for(int id=1;id<=5;++id) {
        auto response=packet(responseId(id),65535,32768,{0});
        assert(signedWord(response,0)==-id);
        assert(static_cast<unsigned char>(response[0])==255);
        assert(static_cast<unsigned char>(response[1])==256-id);
        assert(word(response,2)==65535 && word(response,4)==32768);
        assert(!validRequest(response));
    }
    assert(packet(FE01,1,2,{0})==std::string("\xFF\xFF\x00\x01\x00\x02\x00\x00",8));
    {
        Fixture f;
        for(int id:{0,-32768,6,-6,0xEF01,0xFE01}) {
            auto bad=packet(id,1,0);
            assert(!validRequest(bad));
            assert(f.c.receive(bad,"peer",0).empty());
        }
    }
    const unsigned ids[]={EF01,EF02,EF03,EF04,EF05};
    for(unsigned id:ids) {
        auto p=packet(id,65535,0,id==EF03?std::vector<int>{30,180}:std::vector<int>{});
        assert(p.size()==length(id) && validRequest(p));
        assert(!validRequest(p+std::string(4,'\0')));
        assert(!validRequest(p.substr(0,p.size()-1)));
    }
    assert(packet(EF03,1,0,{30,180})==std::string("\x00\x03\x00\x01\x00\x00\x00\x1E\x00\xB4\x00\x00",12));
    {
        Fixture f;
        auto s=f.send(EF02,1);
        assert(s.size()==12 && signedWord(s,0)==FE02 && word(s,4)==1);
        assert(word(s,6)==CONNECTED && signedWord(s,8)==30 && signedWord(s,10)==180);
        assert(word(f.send(EF03,2,{-1,180}),6)==INVALID);
        assert(word(f.send(EF04,3,{2}),6)==INVALID);
        assert(word(f.send(EF03,4,{31,180}),6)==ACCEPTED);
        assert(f.moves==1 && f.c.moving);
        assert(word(f.send(EF03,5,{32,180}),6)==BUSY && f.moves==1);
        assert(word(f.send(EF04,6,{1}),6)==BUSY);
        assert(word(f.send(EF05,7),6)==BUSY);
        assert(word(f.send(EF04,8,{0}),6)==ACCEPTED);
        f.c.sample(f.c.targetPan,f.c.targetTilt,10);
        f.c.sample(f.c.targetPan,f.c.targetTilt,160);
        assert(!f.c.moving);
    }
    {
        Fixture f;
        auto a=f.send(EF03,65535,{30,181});
        assert(a==f.send(EF03,65535,{30,181}) && f.moves==1);
        assert(word(f.send(EF03,65535,{30,182}),6)==INVALID);
        assert(signedWord(f.send(EF02,0),0)==FE02); // wrap
        assert(word(f.send(EF03,65534,{30,182}),6)==INVALID);
        assert(f.c.receive(packet(EF03,1,0,{30,180}),"other",0).empty());
        assert(word(f.c.receive(packet(EF01,1,0),"other",0),6)==BUSY);
        auto malformed=packet(EF03,1,0,{30,180}); malformed[11]=1;
        assert(word(f.c.receive(malformed,"peer",0),6)==INVALID);
        assert(f.c.receive("bad","peer",0).empty());
    }
    {
        Fixture f;
        f.gpioOK=false;
        assert(word(f.send(EF04,1,{1}),6)==DEVICE_ERROR && f.c.fault);
        assert(word(f.send(EF03,2,{30,180}),6)==DEVICE_ERROR);
        f.gpioOK=true;
        assert(word(f.send(EF04,3,{0}),6)==ACCEPTED && f.c.fault);
        assert(word(f.send(EF02,4),6)&FAULT);
    }
    {
        Fixture f;
        f.c.tick(501);
        assert(f.c.fault && f.holds==1);
        assert(word(f.send(EF05,1,{},501),6)==DEVICE_ERROR);
    }
    {
        Fixture f;
        auto reply=f.send(EF05,1);
        assert(signedWord(reply,0)==FE05 && word(reply,6)==ACCEPTED);
        assert(reply==f.send(EF05,1) && f.moves==1);
        assert(f.c.demo && f.c.moving && !f.c.laser);
        int bp=f.c.basePan, bt=f.c.baseTilt;
        auto path=rasterPath(bp,bt);
        assert(path.size()==24); // 12 horizontal scan rows
        long long now=0;
        for(size_t n=0;n<path.size();++n) {
            assert(f.c.targetPan==path[n].pan && f.c.targetTilt==path[n].tilt);
            assert(f.c.laser==(n!=0)); // approach OFF, all raster segments ON
            assert(std::abs(path[n].pan-bp)*360.0/4096<=1);
            assert(std::abs(path[n].tilt-bt)*360.0/4096<=1);
            f.arrive(now);
        }
        assert(now>3000 && f.c.demo && f.c.moving && !f.c.laser);
        assert(f.c.targetPan==bp && f.c.targetTilt==bt);
        assert(f.c.scanPhase==Controller::ScanPhase::RETURNING);
        f.arrive(now);
        assert(!f.c.demo && !f.c.moving && !f.c.laser);
        assert(f.targets.size()==25); // 24 path points + exact saved tick origin
        for(size_t n=0;n<path.size();n+=2) {
            assert(path[n].tilt==path[n+1].tilt);
            assert(std::abs(path[n].pan-path[n+1].pan)==2*RASTER_HALF_SPAN);
            if(n+2<path.size()) {
                assert(path[n+1].pan==path[n+2].pan); // vertical step, no diagonal cut
                assert(path[n+1].tilt-path[n+2].tilt==RASTER_ROW_STEP);
            }
        }
        assert(path.front().tilt==bt+RASTER_HALF_SPAN);
        assert(path.back().tilt==bt-RASTER_HALF_SPAN);
    }
    {
        Fixture f;
        f.send(EF05,1);
        assert(word(f.send(EF04,2,{0}),6)==ACCEPTED);
        assert(!f.c.demo && !f.c.laser && f.holds==1);
        Fixture edge;
        edge.c.sample(ControlFsm::panTicks(180),0,0);
        assert(word(edge.send(EF05,1),6)==ACCEPTED && !edge.c.laser);
        assert(edge.c.scanPath.size()==14); // 11,9,7,5,3,1,0: include lower edge
    }
    {
        // Time alone never advances an unfinished raster segment, even after 3 seconds.
        Fixture f;
        f.send(EF05,1);
        for(int n=1;n<=20;++n) { f.c.sample(f.c.basePan,f.c.baseTilt,n*200); f.c.tick(n*200); }
        assert(f.moves==1 && f.c.demo && !f.c.laser && !f.c.fault);
        long long now=4000;
        f.arrive(now); assert(f.c.laser && f.moves==2);
        // A target two ticks away is NOT considered arrived (ordinary MOVE allows two).
        f.c.sample(f.c.targetPan-2,f.c.targetTilt,now+50); f.c.tick(now+50);
        f.c.sample(f.c.targetPan-2,f.c.targetTilt,now+200); f.c.tick(now+200);
        assert(f.c.moving && f.moves==2);
        assert(word(f.send(EF05,2,{},now+200),6)==BUSY);
        assert(f.send(EF05,1,{},now+200)==f.c.history.front().response);
        assert(f.moves==2); // replay doesn't restart the path
        assert(word(f.send(EF04,3,{0},now+200),6)==ACCEPTED);
        assert(!f.c.demo && !f.c.laser && f.holds==1);
    }
    {
        // Laser ON failure after approach must not start a scan.
        Fixture f; f.send(EF05,1); f.gpioOK=false;
        long long now=0; f.arrive(now);
        assert(f.c.fault && !f.c.demo && f.moves==1 && f.holds==1);
        // Loss of feedback during scanning shuts output down and cancels the route.
        Fixture lost; lost.send(EF05,1); now=0; lost.arrive(now);
        assert(lost.c.laser); lost.c.tick(now+501);
        assert(lost.c.fault && !lost.c.laser && !lost.c.demo);
        // Every segment still has a 60s fault timeout (no whole-scan 3s limit).
        Fixture stuck; stuck.send(EF05,1);
        for(int n=1;n<=300;++n) {
            stuck.c.sample(stuck.c.basePan,stuck.c.baseTilt,n*200); stuck.c.tick(n*200);
        }
        assert(stuck.c.fault && !stuck.c.demo && !stuck.c.laser);
    }
    {
        // Use raw measured ticks, not rounded whole degrees, including non-integer origins.
        Fixture f; f.c.sample(2049,345,0); f.send(EF05,1);
        assert(f.c.basePan==2049 && f.c.baseTilt==345);
        assert(f.c.targetPan==2038 && f.c.targetTilt==356);
        for(int p:{ControlFsm::PAN_MIN,ControlFsm::PAN_MAX}) assert(!rasterPath(p,345).empty());
        for(int t:{ControlFsm::TILT_MIN,ControlFsm::TILT_MAX}) assert(!rasterPath(2048,t).empty());
        assert(!rasterPath(ControlFsm::PAN_MIN+RASTER_HALF_SPAN,ControlFsm::TILT_MIN+RASTER_HALF_SPAN).empty());
    }
    {
        // Edges, corners and near-edge origins: full completion, legal targets and exact return.
        for(int p:{ControlFsm::PAN_MIN,ControlFsm::PAN_MIN+5,2048,ControlFsm::PAN_MAX})
        for(int t:{-2,-1,0,5,345,ControlFsm::TILT_MAX}) {
            Fixture f; f.c.sample(p,t,0);
            assert(word(f.send(EF05,1),6)==ACCEPTED);
            int originT=std::max(0,t);
            auto path=f.c.scanPath;
            assert(path.front().tilt==std::min(int(ControlFsm::TILT_MAX),originT+RASTER_HALF_SPAN));
            assert(path.back().tilt==std::max(0,originT-RASTER_HALF_SPAN));
            long long now=0;
            for(size_t n=0;n<path.size();++n) {
                assert(path[n].pan>=ControlFsm::PAN_MIN && path[n].pan<=ControlFsm::PAN_MAX);
                assert(path[n].tilt>=0 && path[n].tilt<=ControlFsm::TILT_MAX);
                assert(std::abs(path[n].pan-p)<=RASTER_HALF_SPAN);
                assert(std::abs(path[n].tilt-originT)<=RASTER_HALF_SPAN);
                if(n%2==0) {
                    assert(path[n].tilt==path[n+1].tilt);
                    if(n) {
                        int drop=path[n-1].tilt-path[n].tilt;
                        assert(drop>0 && drop<=RASTER_ROW_STEP);
                        assert(path[n-1].pan==path[n].pan);
                    }
                }
                f.arrive(now);
            }
            assert(!f.c.laser && f.c.targetPan==p && f.c.targetTilt==originT);
            f.arrive(now);
            assert(!f.c.demo && !f.c.moving && !f.c.fault);
        }
        assert(rasterPath(ControlFsm::PAN_MIN-1,0).empty());
        assert(rasterPath(ControlFsm::PAN_MAX+1,0).empty());
        assert(rasterPath(2048,-1).empty());
        assert(rasterPath(2048,ControlFsm::TILT_MAX+1).empty());
    }
    {
        Fixture f;
        f.send(EF03,1,{30,181});
        for(int n=1;n<=300;++n) { f.c.sample(ControlFsm::panTicks(180),ControlFsm::tiltTicks(30),n*200); f.c.tick(n*200); }
        assert(f.c.fault); // move timeout despite fresh stationary samples
        Fixture empty; empty.c.ready=false;
        auto s=empty.send(EF02,1);
        assert(signedWord(s,8)==-32768 && (word(s,6)&FAULT));
    }
    std::cout << "ICD controller tests passed\n";
}
int main(int argc,char**) {
    if(argc==1) { tests(); return 0; }
    // Test bridge: one request hex line -> one C++ controller response hex line.
    Fixture f;
    std::string line;
    while(std::getline(std::cin,line)) {
        std::string bytes;
        for(size_t i=0;i+1<line.size();i+=2) bytes+=char(std::stoul(line.substr(i,2),nullptr,16));
        std::string response=f.c.receive(bytes,"peer",0);
        for(unsigned char c:response) std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);
        std::cout<<std::endl;
    }
}
