#include "laser.h"
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

static int line;

void laserInit(const char* gpiochip, unsigned int offset) {
    int chip = open(gpiochip, O_RDONLY);
    gpiohandle_request request{};
    request.lineoffsets[0] = offset;
    request.flags = GPIOHANDLE_REQUEST_OUTPUT;
    request.default_values[0] = 0;
    request.lines = 1;
    std::strcpy(request.consumer_label, "motor-demo");
    ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &request);
    line = request.fd;
    close(chip);
}

static void setValue(int value) {
    gpiohandle_data data{};
    data.values[0] = value;
    ioctl(line, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
}

void laserOn() { setValue(1); }
void laserOff() { setValue(0); }
