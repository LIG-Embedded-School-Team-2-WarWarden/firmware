#ifndef DEMO_MOTOR_H
#define DEMO_MOTOR_H
struct MotorPosition { int panTicks; int tiltTicks; };
void motorInit(const char* device, unsigned int baud);
void motorMove(int pan, int tilt, int panSpeed, int tiltSpeed);
MotorPosition motorReadPositions();
#endif
