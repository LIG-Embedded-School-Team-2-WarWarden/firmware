#ifndef CONTROL_FSM_H
#define CONTROL_FSM_H
#include <cstdlib>

enum class State { IDLE, AIMING, READY, SHOOT, STOP };

// 순수 상태 전이: 실제 UART/GPIO/UDP 호출은 main.cpp가 담당한다.
struct ControlFsm {
    static const int TOLERANCE = 5;
    static const int STABLE_MS = 150;
    static const int SHOOT_MS = 1000;
    static const int SAMPLE_GAP_MS = 250;
    State state = State::IDLE;
    int pan = 0, tilt = 0;
    long long nearSince = -1, lastSample = -1, shootUntil = 0;

    const char* name() const {
        switch (state) {
        case State::IDLE: return "IDLE";
        case State::AIMING: return "AIMING";
        case State::READY: return "READY";
        case State::SHOOT: return "SHOOT";
        case State::STOP: return "STOP";
        }
        return "IDLE";
    }
    void move(int p, int t) {
        pan = p; tilt = t;
        nearSince = lastSample = -1;
        state = State::AIMING;
    }
    void sample(int p, int t, long long now) {
        if (state != State::AIMING) return;
        bool near = std::abs(p - pan) <= TOLERANCE &&
                    std::abs(t - tilt) <= TOLERANCE;
        if (!near) nearSince = -1;
        else {
            if (nearSince < 0 || lastSample < 0 ||
                now - lastSample > SAMPLE_GAP_MS) nearSince = now;
            if (now - nearSince >= STABLE_MS) state = State::READY;
        }
        lastSample = now;
    }
    void on(long long now) {
        if (state == State::READY) {
            state = State::SHOOT;
            shootUntil = now + SHOOT_MS;
        }
    }
    void off() {
        if (state == State::SHOOT) state = State::IDLE;
    }
    void stop() { state = State::STOP; nearSince = lastSample = -1; }
    void tick(long long now) {
        if (state == State::SHOOT && now >= shootUntil) state = State::IDLE;
    }
};
#endif

