#ifndef ICD_CONTROLLER_H
#define ICD_CONTROLLER_H
#include "protocol.h"
#include "control_fsm.h"
#include "raster_path.h"
#include <functional>
#include <deque>
#include <cmath>
#include <cstdio>
namespace icd {
// All methods run on the UDP/main thread. UART work is submitted asynchronously.
struct Controller {
    std::function<void(int,int)> move;
    std::function<void(int,int)> scanMove;
    std::function<void()> hold;
    std::function<bool(bool)> light;
    std::string owner;
    bool fault=false, laser=false, moving=false, demo=false, ready=false;
    int pan=0, tilt=0, targetPan=0, targetTilt=0, basePan=0, baseTilt=0;
    enum class ScanPhase { NONE, POSITIONING, SCANNING, RETURNING };
    ScanPhase scanPhase=ScanPhase::NONE;
    std::vector<RasterPoint> scanPath;
    size_t scanIndex=0;
    bool preciseMove=false;
    long long sampled=-1, near=-1, moveDeadline=0;
    unsigned tx=0, highest=0;
    bool haveSequence=false;
    struct Record { unsigned seq; std::string request, response; };
    std::deque<Record> history;
    bool setLight(bool on) {
        if(!light(on)) return false;
        laser=on; return true;
    }
    void fail() {
        bool first=!fault;
        fault=true; demo=false; moving=false; scanPhase=ScanPhase::NONE;
        setLight(false);
        if(first) hold();
    }
    void startMove(int p,int t,long long now,bool precise=false) {
        targetPan=p; targetTilt=t; moving=true; near=-1;
        preciseMove=precise;
        moveDeadline=now+ControlFsm::MOVE_MS;
        if(precise) scanMove(p,t); else move(p,t);
    }
    void sample(int p,int t,long long now) {
        if(!ControlFsm::measurementInRange(p,t)) {
            std::fprintf(stderr,"%s\n",ControlFsm::rangeError(p,t).c_str());
            fail(); return;
        }
        if(sampled<0 || now-sampled>ControlFsm::SAMPLE_GAP_MS) near=-1;
        pan=p; tilt=t; sampled=now; ready=true;
        if(moving) {
            int tolerance=preciseMove?int(RASTER_TOLERANCE):ControlFsm::TOLERANCE;
            if(std::abs(p-targetPan)<=tolerance && std::abs(t-targetTilt)<=tolerance) {
                if(near<0) near=now;
                if(now-near>=ControlFsm::STABLE_MS) moving=false;
            } else near=-1;
        }
    }
    void tick(long long now) {
        if(fault) return;
        if(ready && now-sampled>ControlFsm::FRESH_MS) { fail(); return; }
        if(moving && now>=moveDeadline) { fail(); return; }
        if(demo && !moving) {
            if(scanPhase==ScanPhase::RETURNING) {
                demo=false; scanPhase=ScanPhase::NONE;
            } else {
                if(scanPhase==ScanPhase::POSITIONING) {
                    if(!setLight(true)) { fail(); return; }
                    scanPhase=ScanPhase::SCANNING;
                }
                if(scanIndex+1<scanPath.size()) {
                    const auto& point=scanPath[++scanIndex];
                    startMove(point.pan,point.tilt,now,true);
                } else {
                    if(!setLight(false)) { fail(); return; }
                    scanPhase=ScanPhase::RETURNING;
                    startMove(basePan,baseTilt,now,true);
                }
            }
        }
    }
    unsigned flags() const {
        return (!owner.empty()?CONNECTED:0) | (moving?MOVING:0) |
               (laser?LASER:0) | (demo?DEMO:0) | ((fault || !ready)?FAULT:0);
    }
    std::string reply(int id,unsigned seq,const std::vector<int>& values) {
        std::string out=packet(id,tx,seq,values); tx=(tx+1)&65535; return out;
    }
    std::string result(int id,unsigned seq,Result r) {
        return reply(responseId(id),seq,id==EF04?std::vector<int>{r,laser?1:0}:std::vector<int>{r});
    }
    std::string receive(const std::string& bytes,const std::string& peer,long long now) {
        tick(now);
        // Unknown IDs / incomplete headers are dropped, since no response mapping exists.
        if(bytes.size()<6) return {};
        unsigned id=word(bytes,0);
        unsigned seq=word(bytes,2);
        if(responseId(id)==0) return {};
        // FE02 has no result field: malformed status requests are dropped.
        if(!validRequest(bytes)) return id==EF02?std::string{}:result(id,seq,INVALID);
        if(id==EF01) {
            if(!owner.empty() && peer!=owner && (moving || demo || laser)) return result(id,seq,BUSY);
            if(peer!=owner) { history.clear(); haveSequence=false; owner=peer; }
            return result(id,seq,ACCEPTED);
        }
        if(peer!=owner || owner.empty()) return {}; // establish EF01 first
        for(const auto& r:history) if(r.seq==seq) {
            return r.request==bytes?r.response:(id==EF02?std::string{}:result(id,seq,INVALID));
        }
        unsigned delta=(seq-highest)&65535;
        if(haveSequence && (delta==0 || delta>=32768))
            return id==EF02?std::string{}:result(id,seq,INVALID);
        highest=seq; haveSequence=true;
        std::string response;
        if(id==EF02) {
            response=reply(FE02,seq,{int(flags()),ready?int(std::lround(tilt*360.0/4096)): -32768,
                ready?int(std::lround(pan*360.0/4096)): -32768});
        } else {
            Result r=ACCEPTED;
            int t=id==EF03?signedWord(bytes,6):0;
            int p=id==EF03?signedWord(bytes,8):0;
            int on=id==EF04?int(word(bytes,6)):0;
            bool off=id==EF04 && on==0;
            if((id==EF03 && !ControlFsm::validAngles(p,t)) || (id==EF04 && on>1)) r=INVALID;
            else if(off) {
                bool wasDemo=demo; demo=false; scanPhase=ScanPhase::NONE;
                if(wasDemo) { hold(); moving=false; }
                if(!setLight(false)) { fail(); r=DEVICE_ERROR; }
            } else if(fault || !ready) r=DEVICE_ERROR;
            else if(moving || demo) r=BUSY;
            else if(id==EF03) {
                if(!setLight(false)) { fail(); r=DEVICE_ERROR; }
                else startMove(ControlFsm::panTicks(p),ControlFsm::tiltTicks(t),now);
            } else if(id==EF04) {
                if(!setLight(true)) { fail(); r=DEVICE_ERROR; }
            } else if(id==EF05) {
                // Preserve raw feedback, but keep scan/return targets inside travel limits.
                basePan=ControlFsm::clampPan(pan); baseTilt=ControlFsm::clampTilt(tilt);
                scanPath=rasterPath(basePan,baseTilt);
                if(scanPath.empty()) r=INVALID;
                else if(!setLight(false)) { fail(); r=DEVICE_ERROR; }
                else {
                    demo=true; scanIndex=0; scanPhase=ScanPhase::POSITIONING;
                    startMove(scanPath[0].pan,scanPath[0].tilt,now,true);
                }
            }
            response=result(id,seq,r);
        }
        history.push_back(Record{seq,bytes,response});
        if(history.size()>256) history.pop_front();
        return response;
    }
};
}
#endif
