#include "ethernet.h"
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;
static int udp;

void ethernetInit(unsigned short port) {
    udp = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("10.0.0.2"); // 노트북 IP
    server.sin_port = htons(port);               // 노트북 수신 포트

    sendto(udp, "READY", 5, 0,
           reinterpret_cast<const sockaddr*>(&server), sizeof(server));
}

std::string ethernetReceive(Peer& peer) {
    // 명령이 도착할 때까지 기다리고, 송신자의 주소도 함께 저장한다.
    char buffer[128];
    socklen_t length = sizeof(peer.address);
    int size = recvfrom(udp, buffer, sizeof(buffer), 0,
                       reinterpret_cast<sockaddr*>(&peer.address), &length);
    return string(buffer, size);
}

void ethernetReply(const Peer& peer, const string& message) {
    sendto(udp, message.data(), message.size(), 0,
           reinterpret_cast<const sockaddr*>(&peer.address), sizeof(peer.address));
}
