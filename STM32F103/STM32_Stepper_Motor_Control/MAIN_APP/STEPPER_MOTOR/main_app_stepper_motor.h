#pragma once
#include "stdint.h"
#include "stdbool.h"

typedef enum
{
  MOTOR_DIRECTION_Forward = 1,
  MOTOR_DIRECTION_Reverse = -1
} MOTOR_DIRECTION_t;

void Enable_Motor_0(void);
void Disable_Motor_0(void);
void Enable_Motor_1(void);
void Disable_Motor_1(void);
void Set_Motor_Direction_0(MOTOR_DIRECTION_t direction);
void Set_Motor_Direction_1(MOTOR_DIRECTION_t direction);
void Set_PWM_0(uint16_t pwm_value);
void Set_PWM_1(uint16_t pwm_value);
