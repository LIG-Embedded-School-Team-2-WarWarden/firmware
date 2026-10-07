#include "motor.h"
#include "laser.h"
#include "ethernet.h"
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <unistd.h>

struct Command {
    unsigned long sequence;
    int pan, tilt, panSpeed, tiltSpeed;
};

static Command parseCommand(const std::string& message) {
    // MOVE 순번 PAN위치 TILT위치 PAN속도 TILT속도
    std::istringstream input(message);
    std::string name;
    Command command{};
    input >> name >> command.sequence >> command.pan >> command.tilt
          >> command.panSpeed >> command.tiltSpeed;
    return command;
}

static void waitForPosition(const Command& command, const Peer& peer) {
    int stable = 0;
    while (stable < 3) {
        MotorPosition position = motorReadPositions();
        std::printf("PAN=%.2f deg TILT=%.2f deg\n",
                    position.panTicks * 360.0 / 4096,
                    position.tiltTicks * 360.0 / 4096);
        ethernetReply(peer, "POS " + std::to_string(command.sequence) + " " +
                      std::to_string(position.panTicks) + " " +
                      std::to_string(position.tiltTicks));

        // 두 축 모두 목표 오차 5틱 이내인 상태가 3번 이어지면 도착이다.
        bool arrived = std::abs(position.panTicks - command.pan) <= 5 &&
                       std::abs(position.tiltTicks - command.tilt) <= 5;
        stable = arrived ? stable + 1 : 0;
        usleep(50000);
    }
}

int main(int argc, char** argv) {
    // 실행 인자: UART_DEVICE GPIOCHIP LINE BAUD [UDP_PORT]
    unsigned int line = std::atoi(argv[3]);
    unsigned int baud = std::atoi(argv[4]);
    unsigned short port = argc > 5 ? std::atoi(argv[5]) : 5001;
    laserInit(argv[2], line);
    motorInit(argv[1], baud);
    ethernetInit(port);
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    while (true) {
        // 1. UDP 명령을 받는다.
        Peer peer{};
        Command command = parseCommand(ethernetReceive(peer));
        std::string id = std::to_string(command.sequence);
        ethernetReply(peer, "ACCEPTED " + id);

        // 2. 모터를 움직이고 실제 위치가 목표에 도달할 때까지 기다린다.
        motorMove(command.pan, command.tilt, command.panSpeed, command.tiltSpeed);
        waitForPosition(command, peer);
        ethernetReply(peer, "ARRIVED " + id);

        // 3. 레이저를 1초 켰다가 끄고 완료를 알린다.
        laserOn();
        sleep(1);
        laserOff();
        ethernetReply(peer, "DONE " + id);
    }
}
