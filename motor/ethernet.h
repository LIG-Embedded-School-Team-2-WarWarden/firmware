#ifndef DEMO_ETHERNET_H
#define DEMO_ETHERNET_H
#include <string>
#include <netinet/in.h>
struct Peer { sockaddr_in address; };
void ethernetInit(unsigned short port);
std::string ethernetReceive(Peer& peer);
void ethernetReply(const Peer& peer, const std::string& message);
#endif
