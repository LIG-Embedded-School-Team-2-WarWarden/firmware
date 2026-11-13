#include "motor.h"
#include <cstdint>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
using namespace std;
static int uart = -1;
static long long nowMs() {
    return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
}
static bool waitReady(short events, long long deadline, string& error) {
    while (true) {
        long long left = deadline - nowMs();
        if (left <= 0) { error = "UART_TIMEOUT"; return false; }
        pollfd p{uart, events, 0};
        int n = poll(&p, 1, static_cast<int>(left));
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 || (p.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            error = "UART_IO_ERROR"; return false;
        }
        if (n > 0 && (p.revents & events)) return true;
    }
}
static bool transfer(uint8_t* data, int size, bool sending, long long deadline, string& error) {
    int done = 0;
    while (done < size) {
        if (!waitReady(sending ? POLLOUT : POLLIN, deadline, error)) return false;
        ssize_t n = sending ? write(uart, data+done, size-done) : read(uart, data+done, size-done);
        if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
        if (n <= 0) { error = "UART_IO_ERROR"; return false; }
        done += static_cast<int>(n);
    }
    return true;
}
bool motorInit(const char* device, unsigned int baud, string& error) {
    if (uart >= 0) close(uart);
    uart = -1;
    if (baud != 115200) { error = "UNSUPPORTED_BAUD"; return false; }
    uart = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (uart < 0) { error = "UART_OPEN_FAILED"; return false; }
    termios settings{};
    if (tcgetattr(uart, &settings) < 0) { error = "UART_CONFIG_FAILED"; return false; }
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0; settings.c_cc[VTIME] = 0;
    if (cfsetispeed(&settings, B115200) < 0 || cfsetospeed(&settings, B115200) < 0 ||
        tcsetattr(uart, TCSANOW, &settings) < 0 || tcflush(uart, TCIOFLUSH) < 0) {
        error = "UART_CONFIG_FAILED"; return false;
    }
    error.clear(); return true;
}
static uint8_t checksum(const uint8_t* data, int size) {
    unsigned int sum = 0;
    for (int i = 0; i < size; ++i) sum += data[i];
    return static_cast<uint8_t>(~sum);
}
static void word(uint8_t* data, int value) { data[0] = value & 255; data[1] = (value >> 8) & 255; }
bool motorMove(int pan, int tilt, int panSpeed, int tiltSpeed, string& error) {
    uint8_t packet[22] = {255,255,254,18,131,42,6,1,0,0,0,0,0,0,2,0,0,0,0,0,0,0};
    word(packet+8, pan); word(packet+12, panSpeed);
    word(packet+15, tilt); word(packet+19, tiltSpeed);
    packet[21] = checksum(packet+2, 19);
    // 송신 큐 접수까지 확인한다. 다음 UART 패킷도 같은 스트림에서 순서대로 전송된다.
    return transfer(packet, sizeof(packet), true, nowMs()+200, error);
}
bool motorMoveSelected(int pan, int tilt, int speed, bool sendPan, bool sendTilt, string& error) {
    if (!sendPan && !sendTilt) return true;
    uint8_t packet[22] = {255,255,254,0,131,42,6};
    int end = 7;
    for (int id = 1; id <= 2; ++id) {
        if ((id == 1 && !sendPan) || (id == 2 && !sendTilt)) continue;
        packet[end++] = id;
        word(packet+end, id == 1 ? pan : tilt); end += 2;
        word(packet+end, 0); end += 2;
        word(packet+end, speed); end += 2;
    }
    packet[3] = end - 3;
    packet[end] = checksum(packet+2, end-2);
    return transfer(packet, end+1, true, nowMs()+200, error);
}
bool motorReadPositions(MotorPosition& position, string& error) {
    if (tcflush(uart, TCIFLUSH) < 0) { error = "UART_IO_ERROR"; return false; }
    uint8_t request[10] = {255,255,254,6,130,56,8,1,2,54};
    long long deadline = nowMs()+300;
    if (!transfer(request, sizeof(request), true, deadline, error)) return false;
    unsigned int seen = 0;
    MotorPosition result;
    for (int i = 0; i < 2; ++i) {
        uint8_t response[14];
        if (!transfer(response, sizeof(response), false, deadline, error)) return false;
        if (response[0] != 255 || response[1] != 255 || response[3] != 10 ||
            (response[2] != 1 && response[2] != 2) || response[4] != 0 ||
            checksum(response+2, 11) != response[13]) {
            error = "MOTOR_RESPONSE_ERROR"; return false;
        }
        unsigned int bit = 1U << (response[2]-1);
        if (seen & bit) { error = "MOTOR_DUPLICATE_RESPONSE"; return false; }
        seen |= bit;
        int ticks = response[5] | (response[6] << 8);
        if (ticks > 4095) { error = "MOTOR_POSITION_INVALID"; return false; }
        if (response[2] == 1)
            result.panTicks = ticks;
        else
            result.tiltTicks = ticks;
    }
    position = result; error.clear(); return true;
}
