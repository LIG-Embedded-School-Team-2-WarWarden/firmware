#include "controller.h"
#include <cassert>
#include <iostream>
#include <iomanip>
using namespace icd;
struct Fixture {
    Controller c;
    int moves=0,holds=0,calls=0;
    bool gpioOK=true;
    std::vector<scan::Point> targets;
    std::vector<int> scanSpeeds;
    Fixture() {
        c.move=[this](int,int){++moves;};
        c.scanMove=[this](int p,int t,int speed){
            targets.push_back(scan::Point{p,t});
            scanSpeeds.push_back(speed);
        };
        c.hold=[this](){++holds;};
        c.light=[this](bool){++calls;return gpioOK;};
        c.sample(2048,1365,0);
        c.receive(packet(EF01,0,0),"peer",0);
    }
    std::string send(int id,unsigned seq,std::vector<int> data={},long long now=0) {
        return c.receive(packet(id,seq,0,data),"peer",now);
    }
    void arrive(long long& now) {
        now+=50;c.sample(c.targetPan,c.targetTilt,now);c.tick(now);
        now+=150;c.sample(c.targetPan,c.targetTilt,now);c.tick(now);
    }
};
static void tests() {
    {
        Fixture f;
        auto status=f.send(EF02,1);
        assert(signedWord(status,8)==300 && signedWord(status,10)==1800);
        assert(word(f.send(EF03,2,{35,1801}),6)==ACCEPTED);
        assert(f.c.targetTilt==ControlFsm::clampTilt(ControlFsm::tiltTicks(3.5)+ControlFsm::TILT_OFFSET_TICKS));
        assert(f.c.targetPan==ControlFsm::clampPan(ControlFsm::panTicks(180.1)+ControlFsm::PAN_OFFSET_TICKS));
        long long now=0;f.arrive(now);
        status=f.send(EF02,3,{},now);
        assert(signedWord(status,8)==ControlFsm::tiltDecidegrees(f.c.targetTilt));
        assert(signedWord(status,10)==ControlFsm::panDecidegrees(f.c.targetPan));
        Fixture empty;empty.c.ready=false;
        status=empty.send(EF02,1);
        assert(signedWord(status,8)==-32768 && signedWord(status,10)==-32768);
    }
    for(auto values:std::vector<std::vector<int>>{{-300,450},{-1,1800},{0,450},{900,3150}}) {
        Fixture f;assert(word(f.send(EF03,1,values),6)==ACCEPTED);
    }
    for(auto values:std::vector<std::vector<int>>{{-301,1800},{901,1800},{300,449},
                                                {300,3151},{-32768,1800},{30,180}}) {
        Fixture f;assert(word(f.send(EF03,1,values),6)==INVALID && f.moves==0);
    }

    for(int mode:{EF05,EF06,EF07}) {
        Fixture f;
        auto reply=f.send(mode,1);
        assert(word(reply,6)==ACCEPTED && f.c.demo && f.c.laser && f.c.moving);
        assert(f.send(mode,1)==reply && f.targets.size()==1);
        assert(word(f.send(EF03,2,{300,1800}),6)==BUSY);
        const int span=static_cast<int>(std::lround(ControlFsm::DEMO_SCAN_HALF_RANGE_DEG*4096.0/360.0));
        long long now=0;
        for(int i=0;i<=span;++i) {
            assert(f.c.laser && f.scanSpeeds[i]==ControlFsm::DEMO_SPEED);
            assert(f.targets[i].pan==2048+(i%2?span:-span));
            assert(f.targets[i].tilt==1365+span-2*i);
            f.arrive(now);
        }
        assert(f.c.demo && f.c.returning && f.c.moving && !f.c.laser);
        assert(f.c.targetPan==2048 && f.c.targetTilt==1365);
        assert(word(f.send(EF03,3,{300,1800},now),6)==BUSY);
        f.arrive(now);
        assert(!f.c.demo && !f.c.moving && !f.c.laser && !f.c.fault);
        assert(f.c.pan==2048 && f.c.tilt==1365);
    }
    for(int p:{512,3584}) for(int t:{683,2048}) {
        const auto path=scan::path(p,t,11,2);
        assert(!path.empty());
        for(const auto& target:path) assert(ControlFsm::inRange(target.pan,target.tilt));
    }
    for(int completed:{0,1,4}) {
        Fixture f; f.send(EF05,1);
        long long now=0;
        for(int i=0;i<completed;++i) f.arrive(now);
        f.c.sample(2046,1367,now);
        const auto reply=f.send(EF04,2,{0},now);
        assert(word(reply,6)==ACCEPTED && f.c.returning && !f.c.laser);
        assert(f.c.targetPan==2048 && f.c.targetTilt==1365);
        const auto count=f.targets.size();
        assert(f.send(EF04,2,{0},now)==reply && f.targets.size()==count);
        f.send(EF04,3,{0},now);
        assert(f.targets.size()==count);
        f.arrive(now);
        assert(!f.c.demo && !f.c.moving && f.c.pan==2048 && f.c.tilt==1365);
        assert(word(f.send(EF05,4,{},now),6)==ACCEPTED && !f.c.returning && f.c.laser);
    }
    {
        Fixture broken; broken.gpioOK=false;
        assert(word(broken.send(EF05,1),6)==DEVICE_ERROR && broken.c.fault);
        Fixture stale; stale.send(EF05,1); stale.c.tick(501);
        assert(stale.c.fault && !stale.c.laser && !stale.c.demo && stale.holds==1);
    }
    const calibration::Point measured[]={
        {45,0,0},{170,4,2},{180,0,0},{195,-6,-6},{210,-8,-8},{315,0,0}
    };
    calibration::Map map(measured,sizeof(measured)/sizeof(measured[0]));
    assert(map.valid());
    assert(std::abs(map.extraTicks(175).pan-2)<1e-9);
    assert(std::abs(map.extraTicks(175).tilt-1)<1e-9);
    assert(std::abs(map.extraTicks(190).pan+4)<1e-9);
    assert(std::abs(map.extraTicks(200).tilt+6.6666666667)<1e-8);
    assert(map.extraTicks(0).pan==0 && map.extraTicks(360).tilt==0);
    for(int p=600;p<=3400;++p) {
        const auto target=map.apply(p,1200);
        const auto nominal=map.remove(target.pan,target.tilt);
        assert(std::abs(nominal.pan-p)<0.6 && std::abs(nominal.tilt-1200)<0.6);
    }
    {
        Fixture f; f.c.correction=map;
        assert(word(f.send(EF03,1,{0,1900}),6)==ACCEPTED);
        const auto expected=map.apply(ControlFsm::panTicksFromDecidegrees(1900),1024);
        assert(f.c.targetPan==expected.pan && f.c.targetTilt==expected.tilt);
        long long now=0; f.arrive(now);
        const auto status=f.send(EF08,2,{},now);
        const auto nominal=map.remove(expected.pan,expected.tilt);
        assert(signedWord(status,8)-signedWord(status,12)==std::lround(nominal.tilt));
        assert(signedWord(status,10)-signedWord(status,14)==std::lround(nominal.pan));
    }
    {
        // A steep local curve catches missing correction or applying it twice in a scan.
        const calibration::Point points[]={{45,0,0},{179,-4,10},{180,0,0},{181,4,-10},{315,0,0}};
        calibration::Map scanMap(points,5);
        Fixture f; f.c.correction=scanMap;
        const auto center=scanMap.remove(2048,1365);
        f.send(EF05,1);
        const int span=static_cast<int>(std::lround(ControlFsm::DEMO_SCAN_HALF_RANGE_DEG*4096.0/360.0));
        long long now=0;
        for(int i=0;i<=span;++i) {
            const auto expected=scanMap.apply(center.pan+(i%2?span:-span),center.tilt+span-2*i);
            assert(f.c.targetPan==expected.pan && f.c.targetTilt==expected.tilt);
            f.arrive(now);
        }
        assert(f.c.returning && f.c.targetPan==2048 && f.c.targetTilt==1365);
        f.arrive(now);
        assert(!f.c.demo && f.c.pan==2048 && f.c.tilt==1365);
    }
    {
        const calibration::Point bad[]={{180,0,0},{170,0,0}};
        Fixture f; f.c.correction=calibration::Map(bad,2);
        assert(word(f.send(EF05,1),6)==DEVICE_ERROR && f.c.fault && f.targets.empty());
        const calibration::Point reversed[]={{180,0,0},{181,-12,0}};
        assert(!calibration::Map(reversed,2).valid());
    }
    {
        Fixture f;
        f.c.sample(2048,946,0);
        const auto status=f.send(EF08,1);
        assert(status.size()==16 && word(status,0)==FE08);
        assert(signedWord(status,8)==946 && signedWord(status,10)==2048);
        assert(signedWord(status,12)==ControlFsm::TILT_OFFSET_TICKS);
        assert(signedWord(status,14)==ControlFsm::PAN_OFFSET_TICKS);
        f.c.ready=false;
        assert(signedWord(f.send(EF08,2),8)==UNKNOWN_ANGLE);
    }
    {
        Fixture f;f.send(EF03,1,{310,1800});
        assert(word(f.send(EF04,2,{1}),6)==ACCEPTED && f.c.moving && f.c.laser);
        assert(word(f.send(EF05,3),6)==BUSY);
        assert(word(f.send(EF04,4,{0}),6)==ACCEPTED && f.c.moving && !f.c.laser);
        assert(f.c.receive(packet(EF04,5,0,{1}),"other",0).empty());
        assert(word(f.send(EF04,5,{2}),6)==INVALID);
    }

    // Every legal 0.1-degree command round-trips through the encoder grid.
    for(int p=450;p<=3150;++p) {
        int ticks=ControlFsm::panTicksFromDecidegrees(p);
        assert(ControlFsm::panDecidegrees(ticks)==p);
        if(p>450) assert(ticks>ControlFsm::panTicksFromDecidegrees(p-1));
    }
    for(int t=-300;t<=900;++t) {
        int ticks=ControlFsm::tiltTicksFromDecidegrees(t);
        assert(ControlFsm::tiltDecidegrees(ticks)==t);
        assert(ticks==ControlFsm::tiltTicks(t/10.0));
        if(t>-300) assert(ticks>ControlFsm::tiltTicksFromDecidegrees(t-1));
    }
    assert(ControlFsm::tiltTicksFromDecidegrees(-300)==683);
    for(int t:std::vector<int>{-300,-155,-1,0,900}) {
        Fixture f;
        assert(word(f.send(EF03,1,{t,1800}),6)==ACCEPTED);
        const int expected=ControlFsm::clampTilt(ControlFsm::tiltTicks(t/10.0)+ControlFsm::TILT_OFFSET_TICKS);
        assert(f.c.targetTilt==expected);
        long long now=0;f.arrive(now);
        assert(signedWord(f.send(EF02,2,{},now),8)==ControlFsm::tiltDecidegrees(expected));
    }
    for(int ticks:std::vector<int>{679,2052}) {
        Fixture f;f.c.sample(2048,ticks,0);assert(f.c.fault);
    }
    assert(ControlFsm::tiltTicksFromDecidegrees(0)==1024);
    assert(ControlFsm::tiltTicksFromDecidegrees(900)==2048);
    std::cout<<"Demo + calibrated status + decidegree controller tests passed\n";
}
int main(int argc,char**) {
    if(argc==1) {tests();return 0;}
    Fixture f;std::string line;
    while(std::getline(std::cin,line)) {
        std::string bytes;
        for(size_t i=0;i+1<line.size();i+=2) bytes+=char(std::stoul(line.substr(i,2),nullptr,16));
        auto response=f.c.receive(bytes,"peer",0);
        for(unsigned char c:response) std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);
        std::cout<<std::endl;
    }
}

