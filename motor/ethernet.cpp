#include "ethernet.h"
#include <sys/socket.h>

static int udp;

void ethernetInit(unsigned short port) {
    udp = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(port);
    bind(udp, reinterpret_cast<sockaddr*>(&local), sizeof(local));
}

std::string ethernetReceive(Peer& peer) {
    // 명령이 도착할 때까지 기다리고, 송신자의 주소도 함께 저장한다.
    char buffer[128];
    socklen_t length = sizeof(peer.address);
    int size = recvfrom(udp, buffer, sizeof(buffer), 0,
                       reinterpret_cast<sockaddr*>(&peer.address), &length);
    return std::string(buffer, size);
}

void ethernetReply(const Peer& peer, const std::string& message) {
    sendto(udp, message.data(), message.size(), 0,
           reinterpret_cast<const sockaddr*>(&peer.address), sizeof(peer.address));
}
