#ifndef PROCESS_COMMAND_H /* include guards */
#define PROCESS_COMMAND_H

#define MOTOR_MAX 100 // max speed of the motors in percent
#define JOY_MAX 99 // max value you can get from the joystick

#include <Arduino.h>

struct MotorDuty {
  int dutyR;
  int dutyL;
};

MotorDuty processCommand(String cmd);

#endif /* PROCESS_COMMAND_H */