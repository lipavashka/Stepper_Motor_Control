#include "rotary_encoder_execute.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern osMessageQId  MsgBox;

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Waiting_Data(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_ERROR;
  static int32_t queue_tx_step = 0;
  
  taskENTER_CRITICAL();
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  // osDelay(100);
  rotary_encoder->Read_RAW_Encoder_Value = htim2.Instance->CNT;
  if(rotary_encoder->Read_RAW_Encoder_Value > 32768)
  {
    rotary_encoder->RAW_Value = rotary_encoder->Read_RAW_Encoder_Value - 32768;
  }
  else
  {
    rotary_encoder->RAW_Value = 32768 - rotary_encoder->Read_RAW_Encoder_Value;
  }
  htim2.Instance->CNT = 32768;
    
    
    
    
    
    
    
    
    
    
    
    
    
    
  if(rotary_encoder->queue_motor_set_parametrs.Period_0 <= 2000)
  {
    queue_tx_step = 100; // NEMA17-800 NEMA23-2000
  }
  if(rotary_encoder->queue_motor_set_parametrs.Period_0 >= 6000)
  {
    queue_tx_step = -100; // NEMA17-3500 NEMA23-6000
  }
  rotary_encoder->queue_motor_set_parametrs.Period_0 += queue_tx_step;
  taskEXIT_CRITICAL();

  execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_OK;

  return execute_status;
}

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Parse_Data(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_ERROR;

  taskENTER_CRITICAL();
  rotary_encoder->mptr->Enable_0 = rotary_encoder->queue_motor_set_parametrs.Enable_0;
  rotary_encoder->mptr->Enable_1 = rotary_encoder->queue_motor_set_parametrs.Enable_1;
  rotary_encoder->mptr->Direction_0 = rotary_encoder->queue_motor_set_parametrs.Direction_0;
  rotary_encoder->mptr->Direction_1 = rotary_encoder->queue_motor_set_parametrs.Direction_1;
  rotary_encoder->mptr->Period_0 = rotary_encoder->queue_motor_set_parametrs.Period_0;
  rotary_encoder->mptr->Period_1 = rotary_encoder->queue_motor_set_parametrs.Period_1;
  rotary_encoder->mptr->DutyCycle_0 = rotary_encoder->queue_motor_set_parametrs.DutyCycle_0;
  rotary_encoder->mptr->DutyCycle_1 = rotary_encoder->queue_motor_set_parametrs.DutyCycle_1;
   
  rotary_encoder->queue_motor_set_parametrs.counter = rotary_encoder->queue_motor_set_parametrs.counter + 1;
  rotary_encoder->mptr->counter = rotary_encoder->queue_motor_set_parametrs.counter;
  taskEXIT_CRITICAL();

  execute_status = ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_OK;

  return execute_status;
}

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Send_Data_To_Motor_Thread(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_ERROR;

  taskENTER_CRITICAL();
  osMessagePut(MsgBox, (uint32_t)rotary_encoder->mptr, osWaitForever);  // Send Message
  taskEXIT_CRITICAL();

  execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_OK;

  return execute_status;
}