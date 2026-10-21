#include "motor.h"
extern "C" {
int test_motor_open(const char* path){return motorInit(path,115200)?1:0;}
int test_motor_move(int pan,int tilt,int pan_speed,int tilt_speed){return motorMove(pan,tilt,pan_speed,tilt_speed)?1:0;}
void test_motor_close(){motorClose();}
int test_motor_read(int* pan,int* tilt){const auto p=motorReadPositions();*pan=p.panTicks;*tilt=p.tiltTicks;return (p.panValid?1:0)|(p.tiltValid?2:0);}
}
