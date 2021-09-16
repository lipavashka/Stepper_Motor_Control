#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "rotary_encoder_execute.h"

void ROTARY_ENCODER_RUN_IDLE_State(void);
void ROTARY_ENCODER_RUN_INIT_State(void);
void ROTARY_ENCODER_RUN_WAITING_ENCODER_SENSOR_DATA_State(void);
void ROTARY_ENCODER_RUN_PARSE_ENCODER_SENSOR_DATA_State(void);
void ROTARY_ENCODER_RUN_SEND_DATA_TO_MOTOR_THREAD_State(void);
static void ROTARY_ENCODER_Set_State(ROTARY_ENCODER_STATE_t previous, ROTARY_ENCODER_STATE_t current, ROTARY_ENCODER_STATE_t next);

void ROTARY_ENCODER_Init_State_Mashine(void);
void ROTARY_ENCODER_Run_State_Mashine(void);
void  ROTARY_ENCODER_Init_Reference_Motor_Object_Ready(bool (*p_Motor_Object_Ready_Flag)(void));
