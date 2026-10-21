#include "motor.h"
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <cerrno>
#include <chrono>
#include <vector>

static int uart=-1;
using Steady=std::chrono::steady_clock;
static speed_t baudFlag(unsigned baud){switch(baud){case 38400:return B38400;case 57600:return B57600;case 115200:return B115200;case 230400:return B230400;case 460800:return B460800;case 500000:return B500000;case 921600:return B921600;case 1000000:return B1000000;default:return 0;}}
void motorClose(){if(uart>=0)close(uart);uart=-1;}
bool motorInit(const char* device,unsigned baud){
    motorClose();const auto flag=baudFlag(baud);if(!flag)return false;
    uart=open(device,O_RDWR|O_NOCTTY|O_NONBLOCK);if(uart<0)return false;
    termios settings{};if(tcgetattr(uart,&settings)){motorClose();return false;}
    cfmakeraw(&settings);settings.c_cflag&=~(CSIZE|PARENB|CSTOPB|CRTSCTS);settings.c_cflag|=CS8|CLOCAL|CREAD;
    settings.c_cc[VMIN]=0;settings.c_cc[VTIME]=0;cfsetispeed(&settings,flag);cfsetospeed(&settings,flag);
    if(tcsetattr(uart,TCSANOW,&settings)){motorClose();return false;}tcflush(uart,TCIOFLUSH);return true;
}
static bool waitReady(short events,Steady::time_point deadline){
    while(Steady::now()<deadline){pollfd item{uart,events,0};const int n=poll(&item,1,10);
        if(n<0&&errno==EINTR)continue;if(n<0||(item.revents&(POLLERR|POLLHUP|POLLNVAL)))return false;
        if(n>0&&(item.revents&events))return true;}return false;
}
static uint8_t checksum(const uint8_t* bytes,unsigned size){unsigned sum=0;for(unsigned i=0;i<size;++i)sum+=bytes[i];return static_cast<uint8_t>(~sum);}
static void word(uint8_t* out,int value){out[0]=value&255;out[1]=(value>>8)&255;}
static bool uartSend(const uint8_t* data,int size){
    if(uart<0)return false;int sent=0;const auto deadline=Steady::now()+std::chrono::milliseconds(200);
    while(sent<size){if(!waitReady(POLLOUT,deadline))return false;const auto n=write(uart,data+sent,size-sent);
        if(n>0)sent+=n;else if(n<0&&errno!=EINTR&&errno!=EAGAIN)return false;}return true;
}
bool motorMove(int pan,int tilt,int panSpeed,int tiltSpeed){
    // Existing UART broadcast Sync Write: PAN=1, TILT=2, address 42.
    uint8_t packet[22]={255,255,254,18,131,42,6,1,0,0,0,0,0,0,2,0,0,0,0,0,0,0};
    word(packet+8,pan);word(packet+12,panSpeed);word(packet+15,tilt);word(packet+19,tiltSpeed);
    packet[21]=checksum(packet+2,19);return uartSend(packet,sizeof(packet));
}
MotorPosition motorReadPositions(){
    MotorPosition out;if(uart<0)return out;
    // Do not relabel delayed bytes from an older request as a fresh sample.
    tcflush(uart,TCIFLUSH);const uint8_t request[10]={255,255,254,6,130,56,8,1,2,54};
    if(!uartSend(request,sizeof(request)))return out;
    const auto deadline=Steady::now()+std::chrono::milliseconds(200);std::vector<uint8_t> bytes;
    while(!(out.panValid&&out.tiltValid)&&Steady::now()<deadline){
        if(!waitReady(POLLIN,deadline))break;uint8_t incoming[64];const auto n=read(uart,incoming,sizeof(incoming));
        if(n<0&&(errno==EINTR||errno==EAGAIN))continue;if(n<=0)break;bytes.insert(bytes.end(),incoming,incoming+n);
        while(bytes.size()>=14){
            if(bytes[0]!=255||bytes[1]!=255||bytes[3]!=10){bytes.erase(bytes.begin());continue;}
            unsigned sum=0;for(unsigned i=2;i<14;++i)sum+=bytes[i];
            const bool valid=(sum&255)==255 && bytes[4]==0;
            const int ticks=bytes[5]|(bytes[6]<<8);const auto at=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(valid&&bytes[2]==1&&!out.panValid){out.panTicks=ticks;out.panValid=true;out.panUtcMs=at;}
            if(valid&&bytes[2]==2&&!out.tiltValid){out.tiltTicks=ticks;out.tiltValid=true;out.tiltUtcMs=at;}
            bytes.erase(bytes.begin(),bytes.begin()+14);
        }
    }return out;
}
