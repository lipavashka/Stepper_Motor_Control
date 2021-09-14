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

typedef enum
{
  MOTOR_STATUS_ERROR = 0xFF,
} MOTOR_STATUS_t;

typedef enum
{
  MOTOR_STATE_IDLE = 00u,
  MOTOR_STATE_INIT = 01u,
  MOTOR_STATE_WAITING_DATA = 02u,
  MOTOR_STATE_PARSE_DATA = 03u,
  MOTOR_STATE_COMPETE_PARSE_DATA = 04u,  
  MOTOR_STATE_DEFAULT = 0xFEu,
  MOTOR_STATE_ERROR = 0xFFu
} MOTOR_STATE_t;

typedef struct
{
  MOTOR_STATE_t Previous;
  MOTOR_STATE_t Current;
  MOTOR_STATE_t Next;
} MOTOR_STATE_MASHINE_t;

typedef struct
{
  MOTOR_STATUS_t Status;
  MOTOR_STATE_MASHINE_t STATE_MASHINE;
}STEPPER_MOTOR_CONTROL_t;
