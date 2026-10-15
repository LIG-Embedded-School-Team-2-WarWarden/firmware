#include "laser.h"
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

using namespace std;
static int line;

void laserInit(const char* gpiochip, unsigned int offset) {
    int chip = open(gpiochip, O_RDONLY);
    gpiohandle_request request{};
    request.lineoffsets[0] = offset;
    request.flags = GPIOHANDLE_REQUEST_OUTPUT;
    request.default_values[0] = 0;
    request.lines = 1;
    strcpy(request.consumer_label, "motor-demo");
    ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &request);
    line = request.fd;
    close(chip);
}

static bool setValue(int value) {
    gpiohandle_data data{};
    data.values[0] = value;

    int result = ioctl(line, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);

    return result == 0;
}

bool laserOn() {
    return setValue(1);
}

bool laserOff() {
    return setValue(0);
}
