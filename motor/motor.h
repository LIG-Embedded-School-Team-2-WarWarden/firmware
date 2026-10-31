#ifndef DEMO_MOTOR_H
#define DEMO_MOTOR_H
#include <string>
struct MotorPosition { int panTicks = 0; int tiltTicks = 0; };
bool motorInit(const char* device, unsigned int baud, std::string& error);
bool motorMove(int pan, int tilt, int panSpeed, int tiltSpeed, std::string& error);
bool motorMoveSelected(int pan, int tilt, int speed, bool sendPan, bool sendTilt, std::string& error);
bool motorReadPositions(MotorPosition& position, std::string& error);
#endif
