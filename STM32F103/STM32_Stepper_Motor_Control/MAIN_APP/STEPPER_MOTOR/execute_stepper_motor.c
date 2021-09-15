#include "QUEUEs/queue_type.h"
#include "execute_stepper_motor.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
osEvent  rx_evt;

volatile MOTOR_Queue_t MOTOR_Queue_RX;
// osPoolId  mpool;
osMessageQId  MsgBox;
// MOTOR_Queue_t  *rx_rptr;

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
void Set_Motor_Direction_0(QUEUE_MOTOR_DIRECTION_t direction)
{
  switch (direction)
  {
    case(QUEUE_MOTOR_DIRECTION_Forward):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
      break;
    }
    case(QUEUE_MOTOR_DIRECTION_Reverse):
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
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

MOTOR_STATUS_t Execute_Motor_Waiting_Data(STEPPER_MOTOR_CONTROL_t *motor_control)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  static MOTOR_Queue_t  *p_rx_queue_data;

  rx_evt = osMessageGet(MsgBox, osWaitForever);  // wait for message
  if (rx_evt.status == osEventMessage) 
  {
    taskENTER_CRITICAL();
    if(rx_evt.value.p != NULL)
    {
      p_rx_queue_data = rx_evt.value.p;

      MOTOR_Queue_RX.Enable_0 = p_rx_queue_data->Enable_0;
      MOTOR_Queue_RX.Enable_1 = p_rx_queue_data->Enable_1;
      MOTOR_Queue_RX.Direction_0 = p_rx_queue_data->Direction_0;
      MOTOR_Queue_RX.Direction_1 = p_rx_queue_data->Direction_1;
      MOTOR_Queue_RX.Period_0 = p_rx_queue_data->Period_0;
      MOTOR_Queue_RX.Period_1 = p_rx_queue_data->Period_1;
      MOTOR_Queue_RX.DutyCycle_0 = p_rx_queue_data->DutyCycle_0;
      MOTOR_Queue_RX.DutyCycle_1 = p_rx_queue_data->DutyCycle_1;
      MOTOR_Queue_RX.counter = p_rx_queue_data->counter;

      // osPoolFree(mpool, p_rx_queue_data);                  // free memory allocated for message
      HAL_UART_Transmit(&huart3, "Queue Motor RX <<  ", sizeof("Queue Motor RX <<  "), 10);      
      // <- here show message
      HAL_UART_Transmit(&huart3, "\r\n", 2, 10);/**/
      execute_status = MOTOR_STATUS_WAITING_DATA_OK;
    }
    taskEXIT_CRITICAL();
  }
  else
  {
    HAL_UART_Transmit(&huart3, "Queue Motor RX << ERROR\r\n", sizeof("Queue Motor RX << ERROR\r\n"), 10);  
    execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  }  

  return execute_status;
}
    
MOTOR_STATUS_t Execute_Motor_Parse_Data(STEPPER_MOTOR_CONTROL_t *motor_control)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_PARSE_DATA_ERROR;
    
  taskENTER_CRITICAL();
  if(MOTOR_Queue_RX.Enable_0 == true)
  {
    setPeriod(MOTOR_Queue_RX.Period_0); // setPeriod(period_value);
    // set_PWM_0(MOTOR_Queue_RX.DutyCycle_0);
    SET_PWM_0(MOTOR_Queue_RX.DutyCycle_0);
    Set_Motor_Direction_0(MOTOR_Queue_RX.Direction_0);
    Enable_Motor_0();
  }
  else
  {
    Disable_Motor_0();
  }
  taskEXIT_CRITICAL();
  
  execute_status = MOTOR_STATUS_PARSE_DATA_OK;

  return execute_status;
}
    