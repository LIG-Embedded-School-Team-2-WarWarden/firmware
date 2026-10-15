#ifndef DEMO_IO_H
#define DEMO_IO_H
#include <csignal>
#include <cerrno>
#include <chrono>
#include <poll.h>
#include <algorithm>
extern volatile sig_atomic_t stopping;
inline long long nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
// 1=ready, 0=timeout, -1=error/interrupted shutdown.
inline int waitFd(int fd, short events, long long deadline) {
    while (!stopping) {
        long long left = deadline - nowMs();
        if (left <= 0) return 0;
        pollfd item = {fd, events, 0};
        int rc = poll(&item, 1, static_cast<int>(std::min(left, 100LL)));
        if (rc < 0 && errno == EINTR) continue;
        if (rc < 0) return -1;
        if (rc == 0) continue;
        if (item.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
        if (item.revents & events) return 1;
    }
    return -1;
}
#endif
