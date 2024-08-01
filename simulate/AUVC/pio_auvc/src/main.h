#ifndef CAMERA_SUB_H
#define CAMERA_SUB_H

#include "MS5837.h"

const byte mPin1 = 9;
const byte mPin2 = 10;
const byte mPin3 = 12;
const byte mPin4 = 4;
const byte mPin5 = 5;
const byte mPin6 = 6;
const byte LedPin = 26;
const byte camServo = 26;

static float kp = 1.5;
static float ki = 0.00005;
static float kd = 0.00005;

MS5837 sensor;

int setupMotorsAndLed();

#endif // CAMERA_SUB_H
