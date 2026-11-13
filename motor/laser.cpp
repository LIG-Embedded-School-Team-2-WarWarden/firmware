#include "laser.h"
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
static int line = -1;
bool laserInit(const char* gpiochip, unsigned int offset) {
    // RESET에서 OFF 확인에 실패한 핸들은 다시 요청한다.
    if (line >= 0) {
        if (laserOff()) return true;
        close(line);
        line = -1;
    }
    int chip = open(gpiochip, O_RDONLY | O_CLOEXEC);
    if (chip < 0) return false;
    gpiohandle_request request{};
    request.lineoffsets[0] = offset;
    request.flags = GPIOHANDLE_REQUEST_OUTPUT;
    request.default_values[0] = 0;
    request.lines = 1;
    std::strcpy(request.consumer_label, "motor-demo");
    int rc = ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &request);
    close(chip);
    if (rc < 0) return false;
    line = request.fd;
    return laserOff();
}
static bool setValue(int value) {
    if (line < 0) return false;
    gpiohandle_data data{};
    data.values[0] = value;
    return ioctl(line, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) == 0;
}
bool laserOn() { return setValue(1); }
bool laserOff() { return setValue(0); }
