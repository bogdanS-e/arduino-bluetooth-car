#include "getDuty.h"
#include <Arduino.h>

MotorDuty getDuty(String cmd, int motorMax) {
  MotorDuty res = {0, 0};

  if (cmd.length() != 6) {
    return res;
  }

  const char moveDir = cmd[0];
  int moveSpeed = cmd.substring(1, 3).toInt();
  const char turnDir = cmd[3];
  int turnValue = cmd.substring(4, 6).toInt();

  moveSpeed = constrain(moveSpeed, 0, JOY_MAX);
  turnValue = constrain(turnValue, 0, JOY_MAX);

  int move = map(moveSpeed, 0, JOY_MAX, 0, motorMax);
  int turn = map(turnValue, 0, JOY_MAX, 0, motorMax / 2);

  if (move == 0) {
    turn = map(turnValue, 0, JOY_MAX, 0, motorMax / 6);
  }

  if (moveDir == 'B') {
    move = -move;
  }

  if (turnDir == 'R') {
    turn = -turn;
  }

  int dutyR = constrain(move + turn, -motorMax, motorMax);
  int dutyL = constrain(move - turn, -motorMax, motorMax);

  if (moveSpeed == 0 && turnValue == 0) {
    dutyR = 0;
    dutyL = 0;
  }

  res.dutyR = dutyR;
  res.dutyL = dutyL;

  return res;
}