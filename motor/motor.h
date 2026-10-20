#ifndef DEMO_MOTOR_H
#define DEMO_MOTOR_H
struct MotorPosition {
    int panTicks=0, tiltTicks=0;
    bool panValid=false,tiltValid=false;
    long long panUtcMs=0,tiltUtcMs=0;
};
bool motorInit(const char* device, unsigned int baud);
bool motorMove(int pan, int tilt, int panSpeed, int tiltSpeed);
MotorPosition motorReadPositions();
void motorClose();
#endif
