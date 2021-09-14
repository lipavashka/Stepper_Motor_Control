#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stepper_motor_type.h"

static void STEPPER_MOTOR_Set_State(MOTOR_STATE_t previous, MOTOR_STATE_t current, MOTOR_STATE_t next);
void STEPPER_MOTOR_RUN_IDLE_State(void);
void STEPPER_MOTOR_RUN_INIT_State(void);
void STEPPER_MOTOR_RUN_WAITING_DATA_State(void);
void STEPPER_MOTOR_RUN_PARSE_DATA_State(void);
void STEPPER_MOTOR_RUN_COMPLETE_PARSE_DATA_State(STEPPER_MOTOR_CONTROL_t *stepper_motor);
void STEPPER_MOTOR_RUN_DEFAULT_State(STEPPER_MOTOR_CONTROL_t *stepper_motor);
void STEPPER_MOTOR_RUN_ERROR_State(STEPPER_MOTOR_CONTROL_t *stepper_motor);

void STEPPER_MOTOR_Init_State_Mashine(void);
void STEPPER_MOTOR_Run_State_Mashine(void);
