#pragma once

#include "stdint.h"
#include "stdbool.h"

typedef enum
{
  MOTOR_DIRECTION_Forward = 1,
  MOTOR_DIRECTION_Reverse = -1
} MOTOR_DIRECTION_t;
typedef struct
{
  bool Enable_0;
  bool Enable_1;
  MOTOR_DIRECTION_t Direction_0;
  MOTOR_DIRECTION_t Direction_1;
  uint16_t Period_0;
  uint16_t Period_1;
  uint16_t DutyCycle_0;
  uint16_t DutyCycle_1;
  uint32_t counter; 
}MOTOR_Queue_t;
