#include "execute_stepper_motor.h"
#include "stm32f1xx_hal.h"

volatile MOTOR_Queue_t MOTOR_Queue_RX;

extern TIM_HandleTypeDef htim3;

//  STEPPER_MOTOR_Init_State_Mashine

void Enable_Motor_0(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}
void Disable_Motor_0(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}
void Enable_Motor_1(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void Disable_Motor_1(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void Set_Motor_Direction_0(MOTOR_DIRECTION_t direction)
{
  switch (direction)
  {
    case(MOTOR_DIRECTION_Forward):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
      break;
    }
    case(MOTOR_DIRECTION_Reverse):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
      break;
    }
    default:
      while(1);
  } 
}
void Set_Motor_Direction_1(MOTOR_DIRECTION_t direction)
{
  switch (direction)
  {
    case(MOTOR_DIRECTION_Forward):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      break;
    }
    case(MOTOR_DIRECTION_Reverse):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      break;
    }
    default:
      while(1);
  } 
}
/*void Set_PWM_0(uint16_t pwm_value)
{
//  TIM_OC_InitTypeDef sConfigOC;
//
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  sConfigOC.Pulse = pwm_value;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
//  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  /// TIMx->CCR1 = OC_Config->Pulse;
  htim3.Instance->CCR1 = pwm_value;
  
}
void Set_PWM_1(uint16_t pwm_value)
{  
  htim3.Instance->CCR2 = pwm_value;  
}*/
void SET_PWM_0(uint16_t pwm_value)
{ 
  htim3.Instance->CCR1 = pwm_value;  
}
void SET_PWM_1(uint16_t pwm_value)
{  
  htim3.Instance->CCR2 = pwm_value;  
}
void setPeriod(uint16_t period_value)
{  
  // TIMx->ARR
  htim3.Instance->ARR = period_value;  
}
