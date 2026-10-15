#include "motor.h"
#include "laser.h"
#include "ethernet.h"
#include "control_fsm.h"
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <deque>
#include <chrono>
#include <sys/socket.h>

using namespace std;

struct MotorJob {
    bool stop;
    int pan, tilt;
    unsigned long generation;
};

struct Sample {
    MotorPosition position{};
    unsigned long generation = 0;
    unsigned long number = 0;
    long long time = 0;
};

struct Controller {
    ControlFsm fsm;
    Peer control{};
    bool connected = false;
    bool laserIsOn = false;
    unsigned long sequence = 0;
    unsigned long generation = 0;
    unsigned long lastSample = 0;
    int telemetry = -1;
};

// 아래 세 변수만 두 스레드가 함께 사용한다.
static mutex sharedMutex;
static deque<MotorJob> jobs;
static Sample latest;

static long long nowMs() {
    return chrono::duration_cast<chrono::milliseconds>(
        chrono::steady_clock::now().time_since_epoch()).count();
}

// 1. 출력 및 모터 작업 등록
static void publish(const Controller& controller, const string& text) {
    printf("%s\n", text.c_str());
    if (controller.connected) {
        sendto(controller.telemetry, text.data(), text.size(), 0,
               reinterpret_cast<const sockaddr*>(&controller.control.address),
               sizeof(controller.control.address));
    }
}

static bool applyLaser(Controller& controller) {
    bool wanted = controller.fsm.state == State::SHOOT;

    if (wanted == controller.laserIsOn)
        return true;

    bool success;

    if (wanted)
        success = laserOn();
    else
        success = laserOff();

    if (!success)
        return false;

    controller.laserIsOn = wanted;
    return true;
}

static void enqueue(Controller& controller, bool stop, int pan, int tilt) {
    lock_guard<mutex> lock(sharedMutex);
    if (stop) jobs.clear();
    jobs.push_back(MotorJob{stop, pan, tilt, ++controller.generation});
} // 함수가 끝나면 잠금이 풀린다.

// 2. 모터 스레드
static void positionLoop() {
    unsigned long generation = 0, sampleNumber = 0;
    MotorPosition initial = motorReadPositions();
    motorMove(initial.panTicks, initial.tiltTicks, 200, 200);

    while (true) {
        MotorJob job{};
        bool hasJob;
        {
            lock_guard<mutex> lock(sharedMutex);
            hasJob = !jobs.empty();
            if (hasJob) {
                job = jobs.front();
                jobs.pop_front();
            }
        }
        if (hasJob) {
            if (job.stop) {
                MotorPosition position = motorReadPositions();
                motorMove(position.panTicks, position.tiltTicks, 200, 200);
            } else {
                motorMove(job.pan, job.tilt, 200, 200);
            }
            generation = job.generation;
        }

        Sample sample;
        sample.position = motorReadPositions();
        sample.generation = generation;
        sample.number = ++sampleNumber;
        sample.time = nowMs();
        {
            lock_guard<mutex> lock(sharedMutex);
            latest = sample;
        }
        this_thread::sleep_for(chrono::milliseconds(50));
    }
}

// 3. 메인 반복문의 각 단계
static void updateTimerAndLaser(Controller& controller) {
    controller.fsm.tick(nowMs());

    if (!applyLaser(controller))
        printf("GPIO 출력 변경 실패\n");
}

// 새 측정이면 sample에 복사하고 도착 조건을 검사한다.
static bool processLatestSample(Controller& controller, Sample& sample) {
    {
        lock_guard<mutex> lock(sharedMutex);
        sample = latest;
    }
    if (sample.number == controller.lastSample) return false;
    controller.lastSample = sample.number;

    if (sample.generation == controller.generation &&
        nowMs() - sample.time <= ControlFsm::SAMPLE_GAP_MS) {
        controller.fsm.sample(sample.position.panTicks,
                              sample.position.tiltTicks, sample.time);
    }
    return true;
}

static void setControlAddress(Controller& controller, const Peer& sender) {
    controller.control = sender;
    controller.control.address.sin_port = htons(5002);
    controller.connected = true;
}

// 받은 문자열을 해석해 FSM/레이저/작업 큐에 반영한다.
static void handleCommand(Controller& controller, const string& message,
                          const Peer& sender) {
    istringstream input(message);
    string name, extra;
    input >> name;

    if (name == "HELLO" && !(input >> extra)) {
        setControlAddress(controller, sender);
        publish(controller, string("STATE ") + controller.fsm.name());

} else if (name == "MOVE") {
    unsigned long id;
    int pan, tilt, panSpeed, tiltSpeed;

    // 필요한 숫자 다섯 개가 있고, 추가 단어는 없는지 검사
    if (!(input >> id >> pan >> tilt >> panSpeed >> tiltSpeed)
        || (input >> extra)) {
        ethernetReply(sender, "NACK MOVE FORMAT");
        return;
    }

    // 목표 위치 범위 검사
    if (pan < 1366 || pan > 2730 ||
        tilt < 2048 || tilt > 3072) {
        ethernetReply(
            sender,
            "NACK MOVE " + to_string(id) + " RANGE"
        );
        return;
    }

    setControlAddress(controller, sender);
    controller.sequence = id;

    controller.fsm.move(pan, tilt);
    applyLaser(controller);
    enqueue(controller, false, pan, tilt);

    // 작업 큐에 정상적으로 등록한 뒤 접수 응답
    ethernetReply(sender, "ACK MOVE " + to_string(id));
    
    } else if ((name == "Stop" || name == "STOP" || name == "stop") &&
               !(input >> extra)) {
        controller.fsm.stop();
        applyLaser(controller);
        enqueue(controller, true, 0, 0);
    } else if (name == "on" || name == "ON") {
        if (input >> extra) {
            ethernetReply(sender, "NACK ON FORMAT");
            return;
        }
    if (controller.fsm.state != State::READY) {
        ethernetReply(sender, "NACK ON NOT_READY");
        return;
    }

    controller.fsm.on(nowMs());
    applyLaser(controller);

    ethernetReply(sender, "ACK ON");
    } else if (name == "off" || name == "OFF") {
    if (input >> extra) {
        ethernetReply(sender, "NACK OFF FORMAT");
        return;
    }

    // 실제 GPIO OFF 요청
    if (!laserOff()) {
        ethernetReply(sender, "NACK OFF GPIO_ERROR");
        return;
    }

    // OFF 호출이 성공한 뒤 내부 상태 변경
    controller.laserIsOn = false;
    controller.fsm.off();

    ethernetReply(sender, "ACK OFF");
}
}
static void receiveAndHandleCommand(Controller& controller) {
    Peer sender{};
    string message = ethernetReceive(sender, 10);
    handleCommand(controller, message, sender);
}

static void publishUpdates(const Controller& controller, State before,
                           bool fresh, const Sample& sample) {
    if (before != controller.fsm.state)
        publish(controller, string("STATE ") + controller.fsm.name());

    if (fresh) {
        publish(controller, "POS " + to_string(controller.sequence) + " " +
                to_string(sample.position.panTicks) + " " +
                to_string(sample.position.tiltTicks) + " " +
                controller.fsm.name() + " GEN " + to_string(sample.generation));
    }
}

int main(int argc, char** argv) {
    const unsigned int line = 0, baud = 115200;
    unsigned short port = argc > 3 ? atoi(argv[3]) : 5001;
    laserInit(argv[2], line);
    motorInit(argv[1], baud);
    ethernetInit(port);
    setvbuf(stdout, nullptr, _IOLBF, 0);

    Controller controller;
    controller.telemetry = socket(AF_INET, SOCK_DGRAM, 0);
    thread positionThread(positionLoop);
    publish(controller, "STATE IDLE");

    while (true) {
        State before = controller.fsm.state;
        updateTimerAndLaser(controller);

        Sample sample;
        bool fresh = processLatestSample(controller, sample);
        receiveAndHandleCommand(controller);

        updateTimerAndLaser(controller);
        publishUpdates(controller, before, fresh, sample);
    }
}
