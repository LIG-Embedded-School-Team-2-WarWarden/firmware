#ifndef ICD_CONTROLLER_H
#define ICD_CONTROLLER_H
#include "protocol.h"
#include "control_fsm.h"
#include <vector>
#include "calibration.h"
#include <functional>
#include <deque>
#include <cmath>
#include <cstdio>
namespace icd {
// All methods run on the UDP/main thread. UART work is submitted asynchronously.
struct Controller {
    std::function<void(int,int)> move;
    std::function<void(int,int,int)> scanMove;
    std::function<void()> hold;
    std::function<bool(bool)> light;
    std::string owner;
    bool logging=false;
    long long lastPeerDrop=-1;
    void trace(const std::string& text) const {
        if(logging) std::fprintf(stderr,"[FCS] %s\n",text.c_str());
    }
    bool fault=false, laser=false, moving=false, demo=false, ready=false;
    calibration::Map correction;
    int pan=0, tilt=0, targetPan=0, targetTilt=0;
    static const int SCAN_TIMEOUT_MS=15000;
    struct ScanPoint { int pan, tilt; };
    std::vector<ScanPoint> scanPath;
    size_t scanIndex=0;
    bool precise=false;
    // Keep demo active until the saved start position is reached.
    bool returning=false;
    int returnPan=0, returnTilt=0;
    long long sampled=-1, near=-1, moveDeadline=0;
    unsigned tx=0, highest=0;
    bool haveSequence=false;
    struct Record { unsigned seq; std::string request, response; };
    std::deque<Record> history;
    bool setLight(bool on) {
        if(!light(on)) { trace(on?"GPIO_ON_FAILED":"GPIO_OFF_FAILED"); return false; }
        laser=on; return true;
    }
    void fail() {
        bool first=!fault;
        if(first) trace("FAULT_ENTER: output OFF and position hold requested");
        fault=true; demo=false; moving=false; returning=false;
        setLight(false);
        if(first) hold();
    }
    void startMove(int p,int t,long long now,bool scanning=false) {
        targetPan=p; targetTilt=t; moving=true; near=-1;
        precise=scanning;
        moveDeadline=now+(scanning?int(SCAN_TIMEOUT_MS):ControlFsm::MOVE_MS);
        if(scanning) scanMove(p,t,ControlFsm::DEMO_SPEED);
        else move(p,t);
    }
    void buildDemoPath(int centerPan, int centerTilt) {
        // Keep the previous 0.2-degree horizontal half-width (2 encoder ticks).
        constexpr int sideTicks=11, rowOffsetTicks=4, roundTrips=3;
        const int left=ControlFsm::clampPan(centerPan-sideTicks);
        const int right=ControlFsm::clampPan(centerPan+sideTicks);
        scanPath.clear();
        for(int offset : {0, rowOffsetTicks, -rowOffsetTicks}) {
            const int row=ControlFsm::clampTilt(centerTilt+offset);
            scanPath.push_back({left,row});
            for(int trip=0; trip<roundTrips; ++trip) {
                scanPath.push_back({right,row});
                scanPath.push_back({left,row});
            }
        }
    }
    bool returnToStart(long long now) {
        // Both normal completion and OFF cancel use a dark return move.
        if(!setLight(false)) { fail(); return false; }
        if(!returning) {
            returning=true;
            trace("DEMO_RETURN target="+std::to_string(returnPan)+","+std::to_string(returnTilt));
            startMove(returnPan,returnTilt,now,true);
        }
        return true;
    }
    void sample(int p,int t,long long now) {
        if(!correction.valid()) {
            trace("CALIBRATION_INVALID"); fail(); return;
        }
        if(!ControlFsm::measurementInRange(p,t)) {
            std::fprintf(stderr,"%s\n",ControlFsm::rangeError(p,t).c_str());
            fail(); return;
        }
        if(sampled<0 || now-sampled>ControlFsm::SAMPLE_GAP_MS) near=-1;
        pan=p; tilt=t; sampled=now; ready=true;
        if(moving) {
            int tolerance=precise?1:ControlFsm::TOLERANCE;
            if(std::abs(p-targetPan)<=tolerance && std::abs(t-targetTilt)<=tolerance) {
                if(near<0) near=now;
                if(now-near>=ControlFsm::STABLE_MS) moving=false;
            } else near=-1;
        }
    }
    void tick(long long now) {
        if(fault) return;
        if(ready && now-sampled>ControlFsm::FRESH_MS) {
            trace("POSITION_STALE age_ms="+std::to_string(now-sampled)); fail(); return;
        }
        if(moving && now>=moveDeadline) {
            trace("MOVE_TIMEOUT actual="+std::to_string(pan)+","+std::to_string(tilt)+
                  " target="+std::to_string(targetPan)+","+std::to_string(targetTilt)); fail(); return;
        }
        if(!demo) return;
        if(returning) {
            if(!moving) {
                returning=false; demo=false;
                trace("DEMO_COMPLETE: returned to start");
            }
            return;
        }
        // Advance only after feedback confirms arrival at the current endpoint.
        if(!moving) {
            if(scanIndex+1==scanPath.size()) {
                returnToStart(now);
                return;
            }
            const auto& next=scanPath[++scanIndex];
            startMove(next.pan,next.tilt,now,true);
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
        if(bytes.size()<6) { trace("DROP_SHORT_HEADER size="+std::to_string(bytes.size())); return {}; }
        unsigned id=word(bytes,0);
        unsigned seq=word(bytes,2);
        if(responseId(id)==0) { trace("DROP_UNKNOWN_ID value="+std::to_string(id)); return {}; }
        // Status replies have no result field.
        if(!validRequest(bytes)) {
            trace("INVALID_PACKET peer="+peer+" seq="+std::to_string(seq)+" size="+std::to_string(bytes.size()));
            return isStatus(id)?std::string{}:result(id,seq,INVALID);
        }
        if(id==EF01) {
            if(!owner.empty() && peer!=owner && (moving || demo || laser)) {
                trace("CONNECT_BUSY peer="+peer+" owner="+owner); return result(id,seq,BUSY);
            }
            if(peer!=owner) {
                trace("OWNER_CHANGE old="+(owner.empty()?std::string("none"):owner)+" new="+peer);
                history.clear(); haveSequence=false; owner=peer;
            } else trace("CONNECT_SAME_PEER peer="+peer);
            return result(id,seq,ACCEPTED);
        }
        if(peer!=owner || owner.empty()) {
            if(lastPeerDrop<0 || now-lastPeerDrop>=2000) {
                trace("DROP_NOT_OWNER peer="+peer+" owner="+(owner.empty()?std::string("none"):owner)+" seq="+std::to_string(seq));
                lastPeerDrop=now;
            }
            return {};
        }
        for(const auto& r:history) if(r.seq==seq) {
            trace(std::string(r.request==bytes?"REPLAY":"SEQ_CONFLICT")+" peer="+peer+" seq="+std::to_string(seq));
            return r.request==bytes?r.response:(isStatus(id)?std::string{}:result(id,seq,INVALID));
        }
        unsigned delta=(seq-highest)&65535;
        if(haveSequence && (delta==0 || delta>=32768)) {
            trace("OLD_SEQUENCE peer="+peer+" seq="+std::to_string(seq)+" highest="+std::to_string(highest));
            return isStatus(id)?std::string{}:result(id,seq,INVALID);
        }
        highest=seq; haveSequence=true;
        std::string response;
        if(id==EF02) {
            // ICD angles are measured position with motor calibration removed.
            // Keep fractions until the final 0.1-degree wire conversion.
            auto nominal=correction.remove(pan,tilt);
            response=reply(FE02,seq,{int(flags()),
                ready?static_cast<int>(std::lround((nominal.tilt-1024.0)*3600.0/4096.0)):UNKNOWN_ANGLE,
                ready?static_cast<int>(std::lround(nominal.pan*3600.0/4096.0)):UNKNOWN_ANGLE});
        } else if(id==EF08) {
            auto nominal=correction.remove(pan,tilt);
            response=reply(FE08,seq,{int(flags()),ready?tilt:UNKNOWN_ANGLE,ready?pan:UNKNOWN_ANGLE,
                tilt-static_cast<int>(std::lround(nominal.tilt)),pan-static_cast<int>(std::lround(nominal.pan))});
        } else {
            Result r=ACCEPTED;
            int t=id==EF03?signedWord(bytes,6):0;
            int p=id==EF03?signedWord(bytes,8):0;
            int on=id==EF04?int(word(bytes,6)):0;
            bool off=id==EF04 && on==0;
            if((id==EF03 && !ControlFsm::validDecidegrees(p,t)) || (id==EF04 && on>1)) r=INVALID;
            else if(off) {
                if(demo) {
                    if(!returnToStart(now)) r=DEVICE_ERROR;
                } else {
                    demo=false;
                    if(!setLight(false)) { fail(); r=DEVICE_ERROR; }
                }
            } else if(!correction.valid()) {
                trace("CALIBRATION_INVALID"); fail(); r=DEVICE_ERROR;
            } else if(fault || !ready) r=DEVICE_ERROR;
            // Manual illumination is allowed during ordinary movement.
            else if(demo || (moving && id!=EF04)) r=BUSY;
            else if(id==EF03) {
                auto target=correction.apply(ControlFsm::panTicksFromDecidegrees(p),ControlFsm::tiltTicksFromDecidegrees(t));
                startMove(target.pan,target.tilt,now);
            } else if(id==EF04) {
                if(!setLight(true)) { fail(); r=DEVICE_ERROR; }
            } else if(id==EF05 || id==EF06 || id==EF07) {
                int centerPan=ControlFsm::clampPan(pan),centerTilt=ControlFsm::clampTilt(tilt);
                // Require a successful OFF write before sending any movement.
                if(!setLight(false)) { fail(); r=DEVICE_ERROR; }
                else {
                    // Save encoder coordinates once; do not apply offsets again on return.
                    returnPan=centerPan; returnTilt=centerTilt;
                    buildDemoPath(centerPan,centerTilt);
                    demo=true; returning=false; scanIndex=0;
                    trace("DEMO_START points="+std::to_string(scanPath.size())+
                          " center="+std::to_string(centerPan)+","+std::to_string(centerTilt));
                    setLight(true);
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
