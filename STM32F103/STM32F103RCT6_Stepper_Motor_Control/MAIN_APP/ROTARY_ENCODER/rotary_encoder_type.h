#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "QUEUEs/queue_type.h"
#include "arm_math.h"

typedef enum
{
  ROTARY_ENCODER_STATUS_OK                                = 0,
  ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_OK    = 1,
  ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_ERROR = 2,
  ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_OK      = 3,
  ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_ERROR   = 4,
  ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_OK      = 5,
  ROTARY_ENCODER_STATUS_MOTOR_OBJECT_IS_BUSY              = 6,
  ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_ERROR   = 7,
  ROTARY_ENCODER_STATUS_ERROR                             = 0xFF,
}ROTARY_ENCODER_STATUS_t;

typedef enum
{
  ROTARY_ENCODER_STATE_IDLE                        = 0,
  ROTARY_ENCODER_STATE_INIT                        = 1,
  ROTARY_ENCODER_STATE_WAITING_ENCODER_SENSOR_DATA = 2,
  ROTARY_ENCODER_STATE_PARSE_ENCODER_SENSOR_DATA   = 3,
  ROTARY_ENCODER_STATE_SEND_DATA_TO_MOTOR_THREAD   = 4,  
  ROTARY_ENCODER_STATE_DEFAULT                     = 0xFE,
  ROTARY_ENCODER_STATE_ERROR                       = 0xFF
}ROTARY_ENCODER_STATE_t;

typedef struct
{
  ROTARY_ENCODER_STATE_t Previous;
  ROTARY_ENCODER_STATE_t Current;
  ROTARY_ENCODER_STATE_t Next;
}ROTARY_ENCODER_STATE_MASHINE_t;

typedef bool (*Check_Is_Motor_Object_Ready)(void);

typedef struct
{
  const float64_t Rm;
  const float64_t Cm;
  const uint32_t Hols;
  float64_t Cm_SET;
  
  const float64_t Re;
  const float64_t Ce;
  
  const float64_t k_Timer;
  float64_t k_Set;
  float64_t k;
  
  uint32_t number_of_Pulses;
  uint16_t Timer_Period_SET;
}ROTARY_ENCODER_CONVERTER_t;

typedef struct
{
  ROTARY_ENCODER_STATUS_t Status;
  QUEUE_MOTOR_t *mptr;
  QUEUE_MOTOR_t queue_motor_set_parametrs;
  uint32_t Mean_RAW_Encoder_Value;
  uint32_t RAW_Value;
  uint32_t RAW_Value_0;
  uint32_t RAW_Value_1;
  uint32_t RAW_Value_2;
  ROTARY_ENCODER_CONVERTER_t CONVERTER;
  bool Flag_Motor_Object_Ready;
  Check_Is_Motor_Object_Ready Is_Motor_Object_Ready;
  bool Flag_Complete_Encoder_Process;
  ROTARY_ENCODER_STATE_MASHINE_t STATE_MASHINE;
}ROTARY_ENCODER_t;

