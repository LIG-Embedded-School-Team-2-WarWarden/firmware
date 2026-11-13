#include "motor.h"
#include "laser.h"
#include "ethernet.h"
#include "control_fsm.h"
#include "controller.h"
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <deque>
#include <chrono>
#include <csignal>
#include <cctype>
using namespace std;

static long long nowMs() {
    return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
}
// Display the exact application payload. IP/UDP headers are added by the OS.
static void logIcd(const char* direction,const string& peer,const string& bytes) {
    ostringstream out;
    out << "[ICD " << direction << "] peer=" << peer << " bytes=" << bytes.size() << " HEX=";
    out << uppercase << hex << setfill('0');
    for(size_t i=0;i<bytes.size();++i) {
        if(i) out << (i==2 || i==4 || i==6 ? " | " : " ");
        out << setw(2) << static_cast<unsigned>(static_cast<unsigned char>(bytes[i]));
    }
    fprintf(stderr,"%s\n",out.str().c_str());
}
static volatile sig_atomic_t quitting = 0;
static void quitHandler(int) { quitting = 1; }
enum class JobKind { MOVE, SCAN, HOLD, RESET };
struct Job { JobKind kind; int pan, tilt, speed; uint64_t generation; };
struct MotorEvent {
    bool job = false, ok = false;
    JobKind kind = JobKind::MOVE;
    uint64_t generation = 0;
    MotorPosition position;
    long long time = 0;
    string error;
};
static mutex workMutex;
static deque<Job> jobs;
static deque<MotorEvent> events;
static bool workerQuit = false;
static void post(const MotorEvent& event) {
    lock_guard<mutex> lock(workMutex);
    events.push_back(event);
}

// UART는 이 스레드만 사용한다. 모든 읽기/쓰기는 유한한 시간 안에 반환한다.
static void motorLoop(string device) {
    bool healthy = false;
    uint64_t generation = 0;
    while (true) {
        Job job{};
        bool have;
        {
            lock_guard<mutex> lock(workMutex);
            if (workerQuit) break;
            have = !jobs.empty();
            if (have) { job = jobs.front(); jobs.pop_front(); }
        }
        if (have) {
            MotorEvent result;
            result.job = true; result.kind = job.kind; result.generation = job.generation;
            bool ok = true;
            if (job.kind == JobKind::RESET)
                ok = motorInit(device.c_str(), 115200, result.error);
            if (ok && job.kind != JobKind::MOVE && job.kind != JobKind::SCAN) {
                ok = motorReadPositions(result.position, result.error);
                if (ok && job.kind == JobKind::RESET &&
                    !ControlFsm::measurementInRange(result.position.panTicks, result.position.tiltTicks)) {
                    result.error = ControlFsm::rangeError(result.position.panTicks,result.position.tiltTicks); ok = false;
                }
                if (ok) ok = motorMove(ControlFsm::clampPan(result.position.panTicks), ControlFsm::clampTilt(result.position.tiltTicks),
                                       job.speed, job.speed, result.error);
            } else if (ok) {
                // Read before writing so range errors still block the command.
                ok = motorReadPositions(result.position, result.error);
                if (ok && !ControlFsm::measurementInRange(result.position.panTicks, result.position.tiltTicks)) {
                    result.error = ControlFsm::rangeError(result.position.panTicks,result.position.tiltTicks); ok = false;
                }
                if (ok) {
                    // Always send accepted targets: a 0.1-degree move is only 1-2 ticks
                    // and must not be suppressed by the 2-tick arrival tolerance.
                    ok = motorMoveSelected(job.pan, job.tilt, job.speed, true, true, result.error);
                }
            }
            result.ok = ok; result.time = nowMs();
            healthy = ok; generation = job.generation;
            post(result);
        }
        if (healthy) {
            MotorEvent sample;
            sample.generation = generation;
            sample.ok = motorReadPositions(sample.position, sample.error);
            sample.time = nowMs();
            healthy = sample.ok;
            post(sample);
        }
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    // 정상 종료 시에도 가능한 경우 현재 위치 유지 명령을 한 번 시도한다.
    MotorPosition p;
    string error;
    if (healthy && (!motorReadPositions(p, error) ||
        !motorMove(ControlFsm::clampPan(p.panTicks), ControlFsm::clampTilt(p.tiltTicks), ControlFsm::SPEED, ControlFsm::SPEED, error)))
        fprintf(stderr, "Shutdown hold failed: %s\n", error.c_str());
}

int main(int argc,char** argv) {
    if(argc<3 || argc>4) {
        fprintf(stderr,"Usage: %s UART_DEVICE GPIOCHIP [UDP_PORT=2000]\n",argv[0]); return 1;
    }
    int port=argc==4?atoi(argv[3]):2000;
    if(port<1 || port>65535 || !ethernetInit(static_cast<unsigned short>(port))) return 1;
    signal(SIGINT,quitHandler); signal(SIGTERM,quitHandler);
    setvbuf(stderr,nullptr,_IONBF,0);
    fprintf(stderr,"[FCS] START uart=%s gpio=%s tilt_offset_deg=90 angle_unit=0.1deg udp=%d\n",argv[1],argv[2],port);
    icd::Controller app;
    app.logging=true;
    uint64_t generation=0;
    auto queue=[&](JobKind kind,int p,int t,int speed) {
        lock_guard<mutex> lock(workMutex);
        jobs.clear(); jobs.push_back(Job{kind,p,t,speed,++generation});
    };
    app.move=[&](int p,int t) { queue(JobKind::MOVE,p,t,ControlFsm::SPEED); };
    app.scanMove=[&](int p,int t,int speed) { queue(JobKind::SCAN,p,t,speed); };
    app.hold=[&]() { queue(JobKind::HOLD,0,0,ControlFsm::SPEED); };
    app.light=[](bool on) { return on?laserOn():laserOff(); };
    if(!laserInit(argv[2],0)) { fprintf(stderr,"[FCS] GPIO_INIT_FAILED\n"); app.fail(); }
    else queue(JobKind::RESET,0,0,ControlFsm::SPEED);
    thread worker(motorLoop,string(argv[1]));
    while(!quitting) {
        deque<MotorEvent> received;
        { lock_guard<mutex> lock(workMutex); received.swap(events); }
        for(const auto& e:received) {
            if(e.generation!=generation) continue;
            if(!e.ok) { fprintf(stderr,"Motor error: %s\n",e.error.c_str()); app.fail(); }
            else if(!e.job || e.kind==JobKind::RESET) app.sample(e.position.panTicks,e.position.tiltTicks,e.time);
        }
        app.tick(nowMs());
        Peer peer;
        string bytes=ethernetReceive(peer,10);
        if(!bytes.empty()) {
            unsigned address=ntohl(peer.address.sin_addr.s_addr);
            string identity=to_string((address>>24)&255)+"."+to_string((address>>16)&255)+"."+
                to_string((address>>8)&255)+"."+to_string(address&255)+":"+to_string(ntohs(peer.address.sin_port));
            logIcd("RX",identity,bytes);
            {
                fprintf(stderr,"[FCS] RX peer=%s id=%04X seq=%u bytes=%zu flags=%u\n",identity.c_str(),
                    bytes.size()>=2?icd::word(bytes,0):0,bytes.size()>=4?icd::word(bytes,2):0,bytes.size(),app.flags());
                if(bytes.size()==12 && icd::word(bytes,0)==icd::EF03)
                    fprintf(stderr,"[FCS] MOVE elevation=%.1fdeg azimuth=%.1fdeg\n",icd::signedWord(bytes,6)/double(icd::ANGLE_SCALE),icd::signedWord(bytes,8)/double(icd::ANGLE_SCALE));
            }
            string response=app.receive(bytes,identity,nowMs());
            bool sent=false;
            if(!response.empty()) {
                sent=ethernetReply(peer,response);
                logIcd(sent?"TX":"TX_FAILED",identity,response);
                unsigned id=icd::word(response,0);
                fprintf(stderr,"[FCS] %s peer=%s id=%04X tx=%u rx=%u %s=%u\n",sent?"TX":"TX_FAILED",identity.c_str(),id,
                    icd::word(response,2),icd::word(response,4),id==icd::FE02?"flags":"result",icd::word(response,6));
            }
            if(!response.empty() && !sent) fprintf(stderr,"UDP reply failed\n");
        }
    }
    if(!laserOff()) fprintf(stderr,"Laser OFF failed at shutdown\n");
    { lock_guard<mutex> lock(workMutex); workerQuit=true; }
    worker.join(); return 0;
}


