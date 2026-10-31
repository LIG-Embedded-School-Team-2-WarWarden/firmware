#include "ethernet.h"
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
static int udp = -1;
bool ethernetInit(unsigned short port) {
    udp = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (udp < 0) return false;
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(port);
    if (bind(udp, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        close(udp); udp = -1; return false;
    }
    return true;
}
std::string ethernetReceive(Peer& peer, int timeoutMs) {
    pollfd item{udp, POLLIN, 0};
    if (poll(&item, 1, timeoutMs) <= 0 || !(item.revents & POLLIN)) return {};
    char buffer[512];
    socklen_t length = sizeof(peer.address);
    ssize_t size = recvfrom(udp, buffer, sizeof(buffer), MSG_TRUNC,
        reinterpret_cast<sockaddr*>(&peer.address), &length);
    if (size <= 0 || size >= static_cast<ssize_t>(sizeof(buffer))) return {};
    return std::string(buffer, static_cast<size_t>(size));
}
bool ethernetReply(const Peer& peer, const std::string& message) {
    ssize_t size = sendto(udp, message.data(), message.size(), 0,
        reinterpret_cast<const sockaddr*>(&peer.address), sizeof(peer.address));
    return size == static_cast<ssize_t>(message.size());
}
