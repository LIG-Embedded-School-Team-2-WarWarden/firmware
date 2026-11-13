#ifndef CONTROL_FSM_H
#define CONTROL_FSM_H
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace calibration {
// Each row is {command azimuth in degrees, extra pan ticks, extra tilt ticks}.
// Keep azimuths increasing. Add rows here when more measurements are available.
struct Point { double azimuthDeg, panTicks, tiltTicks; };
constexpr Point POINTS[] = {
    {170.0, 0.0, 4.0},
    {180.0, 0.0, 0.0},
    {195.0, 0.0, -2.0},
    {210.0, 0.0, -5.0},
};
}

enum class State { IDLE, AIMING, READY, SHOOT, COMPLETE, STOP, FAIL };
struct ControlFsm {
    static const int SPEED = 2000;
    static constexpr int DEMO_SPEED = 100;
    // EF03 target calibration in encoder ticks. Negative tilt lowers elevation.
    static constexpr int PAN_OFFSET_TICKS = 0;
    static constexpr int TILT_OFFSET_TICKS = -70;
    static constexpr double DEMO_SCAN_HALF_RANGE_DEG = 1;
    static const int PAN_MIN = 512, PAN_MAX = 3584;
    // ICD elevation -30..90 degrees maps to motor 60..180 degrees.
    // ceil(60*4096/360) keeps the lower target inside the mechanical range.
    static const int TILT_MIN = 683, TILT_MAX = 2048;
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
               p >= 45 && p <= 315 && t >= -30 && t <= 90;
    }
    // Wire angles remain integers in units of 0.1 degree throughout the ICD layer.
    static bool validDecidegrees(int p,int t) {
        return p>=450 && p<=3150 && t>=-300 && t<=900;
    }
    static int panTicksFromDecidegrees(int p) {
        return clampPan((p*4096+1800)/3600);
    }
    static int tiltTicksFromDecidegrees(int t) {
        // Apply the offset before division so negative ICD values round correctly.
        return clampTilt(((t+900)*4096+1800)/3600);
    }
    static int roundedDecidegrees(int ticks) {
        int value=ticks*3600;
        return value>=0 ? (value+2048)/4096 : -((-value+2048)/4096);
    }
    static int panDecidegrees(int ticks) { return roundedDecidegrees(ticks); }
    static int tiltDecidegrees(int ticks) { return roundedDecidegrees(ticks-1024); }
    static int panTicks(double degrees) {
        int ticks = static_cast<int>(std::lround(degrees * 4096.0 / 360.0));
        return std::max(int(PAN_MIN), std::min(int(PAN_MAX), ticks));
    }
    static int tiltTicks(double degrees) {
        int ticks = static_cast<int>(std::lround((degrees+90.0) * 4096.0 / 360.0));
        return clampTilt(ticks);
    }
    static bool inRange(int p, int t) {
        return p >= PAN_MIN && p <= PAN_MAX && t >= TILT_MIN && t <= TILT_MAX;
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
