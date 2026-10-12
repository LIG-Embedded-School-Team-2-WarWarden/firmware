#include "motor.h"
#include "laser.h"
#include "ethernet.h"
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <unistd.h>

using namespace std;
struct Command {
    unsigned long sequence;
    int pan, tilt, panSpeed, tiltSpeed;
};

static Command parseCommand(const string& message) {
    istringstream input(message);
    string name;
    Command command{};
    input >> name >> command.sequence >> command.pan >> command.tilt
          >> command.panSpeed >> command.tiltSpeed;
    return command;
}

static void waitForPosition(const Command& command, const Peer& peer) {
    int stable = 0;
    while (stable < 3) {
        MotorPosition position = motorReadPositions();
        printf("PAN=%.2f deg TILT=%.2f deg\n",
                position.panTicks * 360.0 / 4096,
                position.tiltTicks * 360.0 / 4096);
        ethernetReply(peer, "POS " + to_string(command.sequence) + " " +
                      to_string(position.panTicks) + " " +
                      to_string(position.tiltTicks));

        bool arrived = abs(position.panTicks - command.pan) <= 5 &&
                       abs(position.tiltTicks - command.tilt) <= 5;
        stable = arrived ? stable + 1 : 0;
        usleep(50000);
    }
}

int main(int argc, char** argv) {
    // 실행 인자: UART_DEVICE GPIOCHIP LINE BAUD [UDP_PORT]
    unsigned int line = atoi(argv[3]);
    unsigned int baud = atoi(argv[4]);
    unsigned short port = argc > 5 ? atoi(argv[5]) : 5001;
    laserInit(argv[2], line);
    motorInit(argv[1], baud);
    ethernetInit(port);
    setvbuf(stdout, nullptr, _IOLBF, 0);

    while (true) {
        
        Peer peer{};
        Command command = parseCommand(ethernetReceive(peer));
        std::string id = to_string(command.sequence);
        ethernetReply(peer, "ACCEPTED " + id);

        motorMove(command.pan, command.tilt, command.panSpeed, command.tiltSpeed);
        waitForPosition(command, peer);
        ethernetReply(peer, "ARRIVED " + id);
        string msg = ethernetReceive(peer);
        if(msg=="Shoot"){
            laserOn();
            ethernetReceive(peer);
            laserOff();
        }
        ethernetReply(peer, "DONE " + id);
    }
}
