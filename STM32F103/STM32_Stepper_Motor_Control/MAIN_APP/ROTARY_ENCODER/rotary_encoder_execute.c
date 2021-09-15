#include "rotary_encoder_execute.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

// static MOTOR_Queue_t  queue_motor_set_parametrs;

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Waiting_Data(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_ERROR;

  execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_OK;

  return execute_status;
}

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Parse_Data(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_ERROR;

  execute_status = ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_OK;

  return execute_status;
}

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Send_Data_To_Motor_Thread(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_ERROR;
  
  /*taskENTER_CRITICAL();

  // mptr = osPoolAlloc(mpool);                     // Allocate memory for the message
  /if(queue_motor_set_parametrs.Period_0 <= 2000)
  {
    queue_tx_step = 100; // NEMA17-800 NEMA23-2000
  }
  if(queue_motor_set_parametrs.Period_0 >= 6000)
  {
    queue_tx_step = -100; // NEMA17-3500 NEMA23-6000
  }
  queue_motor_set_parametrs.Period_0 += queue_tx_step;
    
  mptr->Enable_0 = queue_motor_set_parametrs.Enable_0;
  mptr->Enable_1 = queue_motor_set_parametrs.Enable_1;
  mptr->Direction_0 = queue_motor_set_parametrs.Direction_0;
  mptr->Direction_1 = queue_motor_set_parametrs.Direction_1;
  mptr->Period_0 = queue_motor_set_parametrs.Period_0;
  mptr->Period_1 = queue_motor_set_parametrs.Period_1;
  mptr->DutyCycle_0 = queue_motor_set_parametrs.DutyCycle_0;
  mptr->DutyCycle_1 = queue_motor_set_parametrs.DutyCycle_1;
   
  queue_motor_set_parametrs.counter = queue_motor_set_parametrs.counter + 1;
  mptr->counter = queue_motor_set_parametrs.counter;
  osMessagePut(MsgBox, (uint32_t)mptr, osWaitForever);  // Send Message

  taskEXIT_CRITICAL();*/

  execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_OK;

  return execute_status;
}