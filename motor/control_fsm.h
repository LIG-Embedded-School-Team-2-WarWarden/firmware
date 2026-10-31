#ifndef CONTROL_FSM_H
#define CONTROL_FSM_H
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

enum class State { IDLE, AIMING, READY, SHOOT, COMPLETE, STOP, FAIL };
struct ControlFsm {
    static const int SPEED = 200;
    static const int PAN_MIN = 1366, PAN_MAX = 2730;
    // Front = 0 ticks; upward increases ticks. floor(60 * 4096 / 360).
    static const int TILT_MIN = 0, TILT_MAX = 682;
    static const int TOLERANCE = 2, STABLE_MS = 150;
    // Measurement envelope only; does not change target or arrival tolerances.
    static const int MEASUREMENT_MARGIN = 3;
    static int clampPan(int p) { return std::max(int(PAN_MIN),std::min(int(PAN_MAX),p)); }
    static int clampTilt(int t) { return std::max(int(TILT_MIN),std::min(int(TILT_MAX),t)); }
    static bool measurementInRange(int p,int t) {
        return p>=PAN_MIN-MEASUREMENT_MARGIN && p<=PAN_MAX+MEASUREMENT_MARGIN &&
               t>=TILT_MIN-MEASUREMENT_MARGIN && t<=TILT_MAX+MEASUREMENT_MARGIN;
    }
    static std::string rangeError(int p,int t) {
        return "POSITION_OUT_OF_RANGE PAN="+std::to_string(p)+" TILT="+std::to_string(t)+
            " PAN_ALLOWED=["+std::to_string(PAN_MIN-MEASUREMENT_MARGIN)+","+std::to_string(PAN_MAX+MEASUREMENT_MARGIN)+
            "] TILT_ALLOWED=["+std::to_string(TILT_MIN-MEASUREMENT_MARGIN)+","+std::to_string(TILT_MAX+MEASUREMENT_MARGIN)+"]";
    }
    static const int SHOOT_MS = 1000, MOVE_MS = 60000;
    static const int FRESH_MS = 500, SAMPLE_GAP_MS = 250;
    State state = State::IDLE;
    std::string fault = "NONE";
    int pan = 0, tilt = 0;
    long long nearSince = -1, lastSample = -1, moveUntil = 0, shootUntil = 0;

    static bool validAngles(double p, double t) {
        return std::isfinite(p) && std::isfinite(t) &&
               p >= 120 && p <= 240 && t >= 0 && t <= 60;
    }
    static int panTicks(double degrees) {
        int ticks = static_cast<int>(std::lround(degrees * 4096.0 / 360.0));
        return std::max(int(PAN_MIN), std::min(int(PAN_MAX), ticks));
    }
    static int tiltTicks(double degrees) {
        int ticks = static_cast<int>(std::lround(degrees * 4096.0 / 360.0));
        return std::max(int(TILT_MIN), std::min(int(TILT_MAX), ticks));
    }
    static bool inRange(int p, int t) {
        // 실제 측정도 같은 범위로 검사한다. 범위 밖으로 자동 복귀하지 않는다.
        return p >= PAN_MIN && p <= PAN_MAX && t >= -TOLERANCE && t <= TILT_MAX;
    }
    const char* name() const {
        switch (state) {
        case State::IDLE: return "IDLE";
        case State::AIMING: return "AIMING";
        case State::READY: return "READY";
        case State::SHOOT: return "SHOOT";
        case State::COMPLETE: return "COMPLETE";
        case State::STOP: return "STOP";
        case State::FAIL: return "FAIL";
        }
        return "FAIL";
    }
    void move(int p, int t, long long now) {
        pan = p; tilt = t; nearSince = lastSample = -1;
        moveUntil = now + MOVE_MS; state = State::AIMING;
    }
    bool sample(int p, int t, long long now) {
        if (state != State::AIMING) return false;
        bool near = std::abs(p-pan) <= TOLERANCE && std::abs(t-tilt) <= TOLERANCE;
        if (!near) nearSince = -1;
        else {
            if (nearSince < 0 || lastSample < 0 || now-lastSample > SAMPLE_GAP_MS)
                nearSince = now;
            if (now-nearSince >= STABLE_MS) state = State::READY;
        }
        lastSample = now;
        return state == State::READY;
    }
    void shoot(long long now) { state = State::SHOOT; shootUntil = now + SHOOT_MS; }
    bool shootExpired(long long now) const { return state == State::SHOOT && now >= shootUntil; }
    bool moveExpired(long long now) const { return state == State::AIMING && now >= moveUntil; }
    void fail(const std::string& reason) { state = State::FAIL; fault = reason; }
    void reset() { state = State::IDLE; fault = "NONE"; nearSince = lastSample = -1; }
};
#endif
