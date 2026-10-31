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
static volatile sig_atomic_t quitting = 0;
static void quitHandler(int) { quitting = 1; }
enum class JobKind { MOVE, SCAN, HOLD, RESET };
struct Job { JobKind kind; int pan, tilt; uint64_t generation; };
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
    int previousPan = 0, previousTilt = 0;
    bool haveTarget = false;
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
                                       ControlFsm::SPEED, ControlFsm::SPEED, result.error);
                if (ok) { previousPan = ControlFsm::clampPan(result.position.panTicks); previousTilt = ControlFsm::clampTilt(result.position.tiltTicks); haveTarget = true; }
            } else if (ok) {
                // Decide from a fresh UART sample, independently for each axis.
                ok = motorReadPositions(result.position, result.error);
                if (ok && !ControlFsm::measurementInRange(result.position.panTicks, result.position.tiltTicks)) {
                    result.error = ControlFsm::rangeError(result.position.panTicks,result.position.tiltTicks); ok = false;
                }
                if (ok) {
                    // Raster row changes are only two ticks; never suppress them as deadband.
                    bool sendPan = job.kind == JobKind::SCAN || abs(result.position.panTicks-job.pan) > ControlFsm::TOLERANCE ||
                        (haveTarget && abs(previousPan-job.pan) > ControlFsm::TOLERANCE);
                    bool sendTilt = job.kind == JobKind::SCAN || abs(result.position.tiltTicks-job.tilt) > ControlFsm::TOLERANCE ||
                        (haveTarget && abs(previousTilt-job.tilt) > ControlFsm::TOLERANCE);
                    ok = motorMoveSelected(job.pan, job.tilt, ControlFsm::SPEED, sendPan, sendTilt, result.error);
                    if (ok) {
                        if (sendPan) previousPan = job.pan;
                        if (sendTilt) previousTilt = job.tilt;
                    }
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
    icd::Controller app;
    uint64_t generation=0;
    auto queue=[&](JobKind kind,int p,int t) {
        lock_guard<mutex> lock(workMutex);
        jobs.clear(); jobs.push_back(Job{kind,p,t,++generation});
    };
    app.move=[&](int p,int t) { queue(JobKind::MOVE,p,t); };
    app.scanMove=[&](int p,int t) { queue(JobKind::SCAN,p,t); };
    app.hold=[&]() { queue(JobKind::HOLD,0,0); };
    app.light=[](bool on) { return on?laserOn():laserOff(); };
    if(!laserInit(argv[2],0)) app.fail();
    else queue(JobKind::RESET,0,0);
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
            string identity=to_string(ntohl(peer.address.sin_addr.s_addr))+":"+to_string(ntohs(peer.address.sin_port));
            string response=app.receive(bytes,identity,nowMs());
            if(!response.empty() && !ethernetReply(peer,response)) fprintf(stderr,"UDP reply failed\n");
        }
    }
    if(!laserOff()) fprintf(stderr,"Laser OFF failed at shutdown\n");
    { lock_guard<mutex> lock(workMutex); workerQuit=true; }
    worker.join(); return 0;
}


