#pragma once
#include "cmsis_os.h"
#include "stdint.h"
#include "stdbool.h"
#include "stepper_motor_type.h"

void Enable_Motor_0(void);
void Disable_Motor_0(void);
void Enable_Motor_1(void);
void Disable_Motor_1(void);
void Set_Motor_Direction_0(QUEUE_MOTOR_DIRECTION_t direction);
void Set_Motor_Direction_1(QUEUE_MOTOR_DIRECTION_t direction);

void SET_PWM_0(uint16_t pwm_value);
void SET_PWM_1(uint16_t pwm_value);

MOTOR_STATUS_t Execute_Motor_Waiting_Data(STEPPER_MOTOR_CONTROL_t *motor_control);
MOTOR_STATUS_t Execute_Motor_Parse_Data(STEPPER_MOTOR_CONTROL_t *motor_control);
