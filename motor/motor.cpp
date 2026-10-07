#include "motor.h"
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

static int uart;

static speed_t baudFlag(unsigned int baud) {
    switch (baud) {
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    }
    return 0;
}

void motorInit(const char* device, unsigned int baud) {
    uart = open(device, O_RDWR | O_NOCTTY);
    termios settings{};
    tcgetattr(uart, &settings);
    cfmakeraw(&settings);
    settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    // 한 바이트 이상 도착할 때까지 read()가 기다린다.
    settings.c_cc[VMIN] = 1;
    settings.c_cc[VTIME] = 0;
    cfsetispeed(&settings, baudFlag(baud));
    cfsetospeed(&settings, baudFlag(baud));
    tcsetattr(uart, TCSANOW, &settings);
    tcflush(uart, TCIOFLUSH);
}

static uint8_t checksum(const uint8_t* bytes, unsigned int size) {
    unsigned int sum = 0;
    for (unsigned int i = 0; i < size; ++i) sum += bytes[i];
    return static_cast<uint8_t>(~sum);
}

// 16비트 값을 하위 바이트, 상위 바이트 순서로 넣는다.
static void word(uint8_t* out, int value) {
    out[0] = value & 255;
    out[1] = (value >> 8) & 255;
}

static void uartSend(const uint8_t* data, int size) {
    int sent = 0;
    while (sent < size) sent += write(uart, data + sent, size - sent);
}

void motorMove(int pan, int tilt, int panSpeed, int tiltSpeed) {
    // Broadcast Sync Write: 주소 42부터 모터별 6바이트(위치, 시간, 속도).
    // PAN ID=1, TILT ID=2. 이동 시간 필드는 0으로 둔다.
    uint8_t packet[22] = {255,255,254,18,131,42,6,
                         1,0,0,0,0,0,0, 2,0,0,0,0,0,0, 0};
    word(packet + 8, pan);
    word(packet + 12, panSpeed);
    word(packet + 15, tilt);
    word(packet + 19, tiltSpeed);
    packet[21] = checksum(packet + 2, 19);
    uartSend(packet, sizeof(packet));
    tcdrain(uart);
}

MotorPosition motorReadPositions() {
    // Sync Read: ID 1, 2의 주소 56부터 각각 8바이트를 요청한다.
    const uint8_t request[10] = {255,255,254,6,130,56,8,1,2,54};
    uartSend(request, sizeof(request));

    MotorPosition position{};
    for (int i = 0; i < 2; ++i) {
        // 정상 응답: FF FF ID LENGTH ERROR DATA[8] CHECKSUM (14바이트).
        // UART는 응답을 나눠 전달할 수 있으므로 14바이트를 모은다.
        uint8_t response[14];
        int received = 0;
        while (received < 14)
            received += read(uart, response + received, 14 - received);
        int ticks = response[5] | (response[6] << 8);
        if (response[2] == 1) position.panTicks = ticks;
        else position.tiltTicks = ticks;
    }
    return position;
}
