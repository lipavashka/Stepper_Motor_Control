
#include "execute_stepper_motor.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef *Debug_UART;
osEvent  rx_evt;
osMessageQId  MsgBox;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void Enable_Motor_0(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
void Disable_Motor_0(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}
void Enable_Motor_1(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void Disable_Motor_1(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void Set_Motor_Direction_0(QUEUE_MOTOR_DIRECTION_t direction)
{
  switch (direction)
  {
    case(QUEUE_MOTOR_DIRECTION_Forward):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      break;
    }
    case(QUEUE_MOTOR_DIRECTION_Reverse):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      break;
    }
    default:
    {
      Disable_Motor_0();
      while(1);
      break;
    }
  } 
}
void Set_Motor_Direction_1(QUEUE_MOTOR_DIRECTION_t direction)
{
  switch (direction)
  {
    case(QUEUE_MOTOR_DIRECTION_Forward):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      break;
    }
    case(QUEUE_MOTOR_DIRECTION_Reverse):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      break;
    }
    default:
    {
      Disable_Motor_1();
      while(1);
      break;
    }
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
  htim2.Instance->CCR4 = pwm_value;  
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
void setPeriod_Motor_0(uint16_t period_value)
{  
  // TIMx->ARR
  htim2.Instance->ARR = period_value;  
}
void CopyData_from_Queue(QUEUE_MOTOR_t *out, const QUEUE_MOTOR_t *in)
{
  out->Enable_0 = in->Enable_0;
  out->Enable_1 = in->Enable_1;
  out->Direction_0 = in->Direction_0;
  out->Direction_1 = in->Direction_1;
  out->Period_0 = in->Period_0;
  out->Period_1 = in->Period_1;
  out->DutyCycle_0 = in->DutyCycle_0;
  out->DutyCycle_1 = in->DutyCycle_1;
  out->counter = in->counter;
}
MOTOR_STATUS_t Execute_Motor_Waiting_Data(STEPPER_MOTOR_CONTROL_t *motor_control)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  static QUEUE_MOTOR_t  *p_rx_queue_data;
  static QUEUE_MOTOR_t  *p_queue_motor_control = NULL;
  static STEPPER_MOTOR_CONTROL_t *p_motor_control = NULL;

  if(motor_control == NULL)
  {
     while(1);
  }
  p_motor_control = motor_control;
  p_queue_motor_control = &motor_control->MOTOR_Queue_RX;
  if(p_queue_motor_control == NULL)
  {
     while(1);
  }
  p_rx_queue_data = NULL;
  p_motor_control->Flag_Object_Ready = true;
  rx_evt = osMessageGet(MsgBox, osWaitForever);  // wait for message
  taskENTER_CRITICAL();
  p_motor_control->Flag_Object_Ready = false;
  if (rx_evt.status == osEventMessage) 
  {
    if(rx_evt.value.p == NULL)
    {
       while(1);
    }
    p_rx_queue_data = rx_evt.value.p;
    // CopyData_from_Queue(&motor_control->MOTOR_Queue_RX, p_rx_queue_data); // <- bug in this function
    CopyData_from_Queue(p_queue_motor_control, p_rx_queue_data);
    HAL_UART_Transmit(Debug_UART, "Queue Motor RX <<  ", sizeof("Queue Motor RX <<  "), 10);      
    // <- here show message
    HAL_UART_Transmit(Debug_UART, "\r\n", 2, 10);
    execute_status = MOTOR_STATUS_WAITING_DATA_OK;
  }
  else
  {
    HAL_UART_Transmit(Debug_UART, "Queue Motor RX << ERROR\r\n", sizeof("Queue Motor RX << ERROR\r\n"), 10);  
    execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  }
    taskEXIT_CRITICAL();
  return execute_status;
}
    
MOTOR_STATUS_t Execute_Motor_Parse_Data(STEPPER_MOTOR_CONTROL_t *motor_control)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_PARSE_DATA_ERROR;

  taskENTER_CRITICAL();
  if(motor_control->MOTOR_Queue_RX.Enable_0 == true)
  {
    if(motor_control->MOTOR_Queue_RX.Period_0 <= 59900)
    {
      setPeriod_Motor_0(motor_control->MOTOR_Queue_RX.Period_0);
      SET_PWM_0(motor_control->MOTOR_Queue_RX.DutyCycle_0);
      Set_Motor_Direction_0(motor_control->MOTOR_Queue_RX.Direction_0);
      Enable_Motor_0();
    }
    else
    {
      Disable_Motor_0();
    }
  }
  else
  {
    Disable_Motor_0();
  }
  taskEXIT_CRITICAL();
  
  
  execute_status = MOTOR_STATUS_PARSE_DATA_OK;

  return execute_status;
}
    