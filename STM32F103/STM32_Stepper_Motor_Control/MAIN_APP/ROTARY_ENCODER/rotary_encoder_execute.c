#include "rotary_encoder_execute.h"
#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

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

  execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_OK;

  return execute_status;
}