#ifndef GET_DUTY_H
#define GET_DUTY_H

#define JOY_MAX 99 // max value you can get from the joystick

#include <Arduino.h>

struct MotorDuty {
  int dutyR;
  int dutyL;
};

MotorDuty getDuty(String cmd, int motorMax);

#endif