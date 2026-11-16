/* protos.h - prototypes for functions that use default arguments.
 * The Arduino IDE's automatic prototype generator cannot handle default
 * arguments, so these are declared here explicitly (definitions carry no defaults). */
#ifndef PROTOS_H
#define PROTOS_H
#include <stdint.h>
#include "config.h"
void readUltrasonics(bool median = false);
void imuCalibrate(int n = 800);
float driveStraight(float mm, int speed, float head, bool wallCentre = false, int frontStopMM = 0, int *tileOut = 0);
void driveToPoint(float x, float y, int speed, int frontStopMM = 0);
int followLine(int speed, float maxMM = 5000, int frontStopMM = 0, bool stopOnJunction = true);
int toJunction(int speed = SPEED_NORMAL, float maxMM = 1500);
void passJunctions(int n, int speed = SPEED_NORMAL);
void beep(int n, int ms = 80);
#endif
