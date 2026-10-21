// Standalone automatic PAN/TILT sweep. No UDP or GPIO/laser access.
// Same little-endian position/speed register layout as firmware/motor/motor.cpp.
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

static volatile sig_atomic_t stopping = 0;
static void stopSignal(int) { stopping = 1; }
static long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Position { int pan, tilt; };
struct Options {
    std::string device = "/dev/ttyPS1";
    Position a{1920, 2432}, b{2176, 2688};
    int speed = 200, pauseMs = 1000, moveTimeoutMs = 15000, cycles = 0;
};
static void help() {
    puts("Usage: motor-sweep [options]\n"
         "  --device PATH    UART device, default /dev/ttyPS1 (115200 8N1)\n"
         "  --a PAN TILT     endpoint A, default 1920 2432\n"
         "  --b PAN TILT     endpoint B, default 2176 2688\n"
         "  --speed N        speed of both motors, 1..1000, default 200\n"
         "  --pause-ms N     dwell after arrival, default 1000\n"
         "  --timeout-ms N   per-move deadline, default 15000\n"
         "  --cycles N       A then B = one cycle; 0 = repeat forever (default)\n"
         "Positions are ticks, NOT degrees. PAN 1366..2730; TILT 2048..3072.\n"
         "Requires responding motor IDs 1 (PAN) and 2 (TILT).\n"
         "Starts moving on execution. Stop other UART users first.\n"
         "Ctrl+C ends the sweep and attempts to hold freshly measured positions.");
}
static int integer(const char* text) {
    if (!text[0] || std::strlen(text) > 8 || std::strspn(text, "0123456789") != std::strlen(text))
        throw std::runtime_error("Expected a non-negative decimal integer");
    return static_cast<int>(std::strtol(text, nullptr, 10));
}
static bool validTarget(Position p) {
    return p.pan >= 1366 && p.pan <= 2730 && p.tilt >= 2048 && p.tilt <= 3072;
}
static uint8_t checksum(const uint8_t* data, size_t size) {
    unsigned int sum = 0;
    for (size_t i = 0; i < size; ++i) sum += data[i];
    return static_cast<uint8_t>(~sum);
}
static void word(uint8_t* dest, int value) {
    dest[0] = static_cast<uint8_t>(value);
    dest[1] = static_cast<uint8_t>(value >> 8);
}

class Serial {
    int fd = -1;
    bool restore = false;
    termios saved{};
    void wait(short event, long long deadline, bool honorStop) {
        while (true) {
            if (honorStop && stopping) throw std::runtime_error("Interrupted");
            long long left = deadline - nowMs();
            if (left <= 0) throw std::runtime_error("UART timeout");
            pollfd item{fd, event, 0};
            int rc = poll(&item, 1, static_cast<int>(std::min(left, 50LL)));
            if (rc < 0 && errno == EINTR) continue;
            if (rc < 0) throw std::runtime_error(std::string("poll: ") + std::strerror(errno));
            if (rc == 0) continue;
            if (item.revents & event) return;
            if (item.revents & (POLLERR | POLLHUP | POLLNVAL))
                throw std::runtime_error("UART disconnected/unavailable");
        }
    }
public:
    ~Serial() {
        if (fd >= 0) {
            if (restore && tcsetattr(fd, TCSANOW, &saved) < 0) perror("restore UART");
            close(fd);
        }
    }
    void openDevice(const std::string& path) {
        fd = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) throw std::runtime_error(std::string("open UART: ") + std::strerror(errno));
        if (tcgetattr(fd, &saved) < 0) throw std::runtime_error("tcgetattr failed");
        restore = true;
        termios config = saved;
        cfmakeraw(&config);
        config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        config.c_cflag |= CS8 | CLOCAL | CREAD;
        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 0;
        if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0 ||
            tcsetattr(fd, TCSANOW, &config) < 0)
            throw std::runtime_error("UART setup failed");
    }
    void send(const uint8_t* data, size_t size, bool honorStop = true) {
        long long deadline = nowMs() + 300;
        size_t sent = 0;
        while (sent < size) {
            wait(POLLOUT, deadline, honorStop);
            ssize_t n = write(fd, data+sent, size-sent);
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (n <= 0) throw std::runtime_error("UART write failed");
            sent += static_cast<size_t>(n);
        }
    }
    int readPosition(uint8_t id, bool honorStop = true) {
        // Individual READ of address 56, two bytes; no Sync Read dependency.
        uint8_t query[] = {255, 255, id, 4, 2, 56, 2, 0};
        query[7] = checksum(query+2, 5);
        if (tcflush(fd, TCIFLUSH) < 0) throw std::runtime_error("UART flush failed");
        send(query, sizeof(query), honorStop);
        std::vector<uint8_t> bytes;
        long long deadline = nowMs() + 300;
        try {
            while (bytes.size() < 4096) {
                wait(POLLIN, deadline, honorStop);
                uint8_t block[128];
                ssize_t n = read(fd, block, sizeof(block));
                if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
                if (n <= 0) throw std::runtime_error("UART read failed");
                bytes.insert(bytes.end(), block, block+n);
                while (bytes.size() >= 4) {
                    if (bytes[0] != 255 || bytes[1] != 255 || bytes[3] < 2) {
                        bytes.erase(bytes.begin()); continue;
                    }
                    size_t size = static_cast<size_t>(bytes[3]) + 4;
                    // Only an 8-byte response (or echoed query) can satisfy this read.
                    if (bytes[2] != id || bytes[3] != 4) {
                        bytes.erase(bytes.begin()); continue;
                    }
                    if (bytes.size() < size) break;
                    if (checksum(bytes.data()+2, size-3) != bytes[size-1]) {
                        bytes.erase(bytes.begin()); continue;
                    }
                    if (std::equal(query, query+sizeof(query), bytes.begin())) {
                        bytes.erase(bytes.begin(), bytes.begin()+size); continue;
                    }
                    if (bytes[4] != 0)
                        throw std::runtime_error("Servo status error " + std::to_string(bytes[4]));
                    return bytes[5] | (bytes[6] << 8);
                }
            }
            throw std::runtime_error("Excessive unexpected UART data");
        } catch (const std::exception& e) {
            throw std::runtime_error("ID " + std::to_string(id) + ": " + e.what());
        }
    }
    Position readBoth(bool honorStop = true) {
        int pan = readPosition(1, honorStop);
        int tilt = readPosition(2, honorStop);
        return Position{pan, tilt};
    }
    void move(Position p, int speed, bool honorStop = true) {
        uint8_t packet[22] = {255,255,254,18,131,42,6,
                             1,0,0,0,0,0,0, 2,0,0,0,0,0,0, 0};
        word(packet+8, p.pan);
        word(packet+12, speed);
        word(packet+15, p.tilt);
        word(packet+19, speed);
        packet[21] = checksum(packet+2, 19);
        send(packet, sizeof(packet), honorStop);
    }
};
static void pauseMs(int duration) {
    long long deadline = nowMs() + duration;
    while (!stopping) {
        long long left = deadline - nowMs();
        if (left <= 0) break;
        poll(nullptr, 0, static_cast<int>(std::min(left, 50LL)));
    }
}
static void goTo(Serial& uart, Position target, const Options& options) {
    uart.move(target, options.speed);
    printf("MOVE PAN=%d TILT=%d SPEED=%d\n", target.pan, target.tilt, options.speed);
    long long deadline = nowMs() + options.moveTimeoutMs;
    int stable = 0;
    while (!stopping) {
        Position actual = uart.readBoth();
        if (std::abs(actual.pan-target.pan) <= 5 && std::abs(actual.tilt-target.tilt) <= 5)
            ++stable;
        else stable = 0;
        if (stable >= 3) {
            printf("ARRIVED PAN=%d TILT=%d\n", actual.pan, actual.tilt);
            pauseMs(options.pauseMs);
            return;
        }
        if (nowMs() >= deadline) throw std::runtime_error("Movement timeout; stopping sweep");
        pauseMs(50);
    }
}
int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    Options options;
    try {
        for (int i = 1; i < argc; ++i) {
            std::string flag = argv[i];
            if (flag == "--help" || flag == "-h") { help(); return 0; }
            if (i+1 >= argc) throw std::runtime_error("Missing option value");
            if (flag == "--device") options.device = argv[++i];
            else if (flag == "--a" || flag == "--b") {
                if (i+2 >= argc) throw std::runtime_error("Endpoint needs PAN and TILT");
                int pan = integer(argv[++i]);
                int tilt = integer(argv[++i]);
                if (flag == "--a") options.a = Position{pan, tilt};
                else options.b = Position{pan, tilt};
            } else {
                int value = integer(argv[++i]);
                if (flag == "--speed") options.speed = value;
                else if (flag == "--pause-ms") options.pauseMs = value;
                else if (flag == "--timeout-ms") options.moveTimeoutMs = value;
                else if (flag == "--cycles") options.cycles = value;
                else throw std::runtime_error("Unknown option: " + flag);
            }
        }
        if (!validTarget(options.a) || !validTarget(options.b))
            throw std::runtime_error("Endpoint out of range: PAN 1366..2730, TILT 2048..3072");
        if (options.a.pan == options.b.pan && options.a.tilt == options.b.tilt)
            throw std::runtime_error("A and B must differ on at least one axis");
        if (options.speed < 1 || options.speed > 1000 || options.pauseMs > 60000 ||
            options.moveTimeoutMs < 100 || options.moveTimeoutMs > 120000)
            throw std::runtime_error("Invalid speed/pause/timeout; use --help");
    } catch (const std::exception& e) {
        fprintf(stderr, "ERROR: %s\n", e.what()); return 1;
    }
    struct sigaction action{};
    action.sa_handler = stopSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    Serial uart;
    bool mayHaveMoved = false;
    int result = 0;
    try {
        uart.openDevice(options.device);
        Position current = uart.readBoth();
        printf("READY: PAN=%d TILT=%d; A=(%d,%d), B=(%d,%d), 115200 baud\n",
               current.pan, current.tilt, options.a.pan, options.a.tilt, options.b.pan, options.b.tilt);
        unsigned long completed = 0;
        while (!stopping && (!options.cycles || completed < static_cast<unsigned long>(options.cycles))) {
            mayHaveMoved = true;
            goTo(uart, options.a, options);
            if (stopping) break;
            goTo(uart, options.b, options);
            if (stopping) break;
            ++completed;
            printf("CYCLE %lu COMPLETE\n", completed);
        }
    } catch (const std::exception& e) {
        if (!stopping) { fprintf(stderr, "ERROR: %s\n", e.what()); result = 1; }
    }
    if (mayHaveMoved) {
        try {
            // Bounded fresh reads during cleanup; do not use a stale cached position.
            Position current = uart.readBoth(false);
            uart.move(current, options.speed, false);
            printf("HOLD requested at PAN=%d TILT=%d (torque setting unchanged)\n", current.pan, current.tilt);
        } catch (const std::exception& e) {
            fprintf(stderr, "HOLD NOT CONFIRMED: %s; motor may still be moving to its last target\n", e.what());
            result = 1;
        }
    }
    puts("Sweep ended.");
    return result ? result : (stopping ? 130 : 0);
}
