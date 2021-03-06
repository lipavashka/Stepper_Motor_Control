#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "QUEUEs/queue_type.h"

typedef enum
{
  MOTOR_STATUS_OK = 0,
  MOTOR_STATUS_WAITING_DATA_OK = 1,
  MOTOR_STATUS_WAITING_DATA_ERROR = 2,
  MOTOR_STATUS_PARSE_DATA_OK = 3,
  MOTOR_STATUS_PARSE_DATA_ERROR = 4,
  MOTOR_STATUS_ERROR = 0xFF,
} MOTOR_STATUS_t;

typedef enum
{
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_INIT = 1,
  MOTOR_STATE_WAITING_DATA = 2,
  MOTOR_STATE_PARSE_DATA = 3, 
  MOTOR_STATE_DEFAULT = 0xFE,
  MOTOR_STATE_ERROR = 0xFF
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
  QUEUE_MOTOR_t MOTOR_Queue_RX;
  bool Flag_Object_Ready;
  bool Flag_Complete_Motor_Process;
  MOTOR_STATE_MASHINE_t STATE_MASHINE;
}STEPPER_MOTOR_CONTROL_t;
