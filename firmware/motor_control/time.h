#include "TickTwo.h"

void setMotorUpdateFlag();

bool MOTOR_UPDATE_FLAG = false;

void setMotorUpdateFlag() {
  MOTOR_UPDATE_FLAG = true;
}
