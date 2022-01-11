#include "rotary_encoder_execute.h"
#include "stm32f1xx_hal.h"
#include "APPLICATION_SETTINGS.h"

extern UART_HandleTypeDef *Debug_UART;
extern TIM_HandleTypeDef *Encoder_Timer;
extern osMessageQId  MsgBox;
extern void Error_Handler(void);

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Waiting_Data(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_ERROR;
  // static int32_t queue_tx_step = 0;
  // static uint32_t RAW_Value_0 = 0;
  // static uint32_t RAW_Value_1 = 0;
  // static uint32_t RAW_Value_2 = 0;
  
  taskENTER_CRITICAL();
  


  rotary_encoder->RAW_Value_0 = rotary_encoder->RAW_Value_1;
  rotary_encoder->RAW_Value_1 = rotary_encoder->RAW_Value_2;
  rotary_encoder->RAW_Value_2 = Encoder_Timer->Instance->CNT;
  rotary_encoder->Mean_RAW_Encoder_Value = (rotary_encoder->RAW_Value_0 + rotary_encoder->RAW_Value_1 + rotary_encoder->RAW_Value_2) / 3;
  if(rotary_encoder->Mean_RAW_Encoder_Value > 32768)
  {
    rotary_encoder->RAW_Value = 0;
  }
  else
  {
    rotary_encoder->RAW_Value = 32768 - rotary_encoder->Mean_RAW_Encoder_Value;
    rotary_encoder->CONVERTER.number_of_Pulses =  rotary_encoder->RAW_Value;
  }

  rotary_encoder->RAW_Value = 0;
  Encoder_Timer->Instance->CNT = 32768;   


    
  /*if(rotary_encoder->queue_motor_set_parametrs.Period_0 <= 2000)
  {
    queue_tx_step = 100; // NEMA17-800 NEMA23-2000
  }
  if(rotary_encoder->queue_motor_set_parametrs.Period_0 >= 3500//6000//)
  {
    queue_tx_step = -100; // NEMA17-3500 NEMA23-6000
  }
  rotary_encoder->queue_motor_set_parametrs.Period_0 += queue_tx_step; 
  */

  /*rotary_encoder->queue_motor_set_parametrs.Period_0 = rotary_encoder->RAW_Value * 10;
  if(rotary_encoder->queue_motor_set_parametrs.Period_0 <= 40000)
  {
    rotary_encoder->queue_motor_set_parametrs.Period_0 = 40000 - rotary_encoder->queue_motor_set_parametrs.Period_0;
  }
  else
  {
    rotary_encoder->queue_motor_set_parametrs.Period_0 = 40000;
  }*/
  
  // rotary_encoder->queue_motor_set_parametrs.Period_0 = 56250;
  
  
  taskEXIT_CRITICAL();

  execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_OK;

  return execute_status;
}

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Parse_Data(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_ERROR;

  taskENTER_CRITICAL();
  
  rotary_encoder->CONVERTER.number_of_Pulses = (rotary_encoder->CONVERTER.number_of_Pulses / 2);
  
  if((rotary_encoder->CONVERTER.number_of_Pulses != 0) && (rotary_encoder->CONVERTER.number_of_Pulses > 0))
  {
//    if(rotary_encoder->CONVERTER.number_of_Pulses > 50)
//    {
//      rotary_encoder->CONVERTER.number_of_Pulses = 50;
//    }
//    else if(rotary_encoder->CONVERTER.number_of_Pulses < 2)
//    {
//      rotary_encoder->CONVERTER.number_of_Pulses = 2;
//    }
//    else
//    {
//      ;
//    }
    rotary_encoder->CONVERTER.k_Set = (rotary_encoder->CONVERTER.Cm_SET / rotary_encoder->CONVERTER.Cm);
    if(rotary_encoder->CONVERTER.Ce == 0)
    {
      Error_Handler();
    }
    rotary_encoder->CONVERTER.k = (((rotary_encoder->CONVERTER.Cm / rotary_encoder->CONVERTER.Ce) * rotary_encoder->CONVERTER.k_Set) * (float64_t)rotary_encoder->CONVERTER.Hols);
    float64_t _timer_Period_SET = ( ( (float64_t)((float64_t)os_delay_rotary_encoder/*mSec*/ / (float64_t)rotary_encoder->CONVERTER.number_of_Pulses) * rotary_encoder->CONVERTER.k_Timer) * rotary_encoder->CONVERTER.k );
    _timer_Period_SET = (_timer_Period_SET * (float64_t)K_rotary_encoder); // 2.5
    rotary_encoder->CONVERTER.Timer_Period_SET = (uint16_t)_timer_Period_SET;
    
    rotary_encoder->queue_motor_set_parametrs.Period_0 = rotary_encoder->CONVERTER.Timer_Period_SET;
    rotary_encoder->queue_motor_set_parametrs.Enable_0 = true;
    
    rotary_encoder->queue_motor_set_parametrs.Period_1 = rotary_encoder->CONVERTER.Timer_Period_SET;
    rotary_encoder->queue_motor_set_parametrs.Enable_1 = true;
  }
  else
  {
    rotary_encoder->queue_motor_set_parametrs.Enable_0 = false;
    rotary_encoder->queue_motor_set_parametrs.Enable_1 = false;
  }
  
  
  
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

bool ROTARY_ENCODER_Is_Motor_Object_Ready(ROTARY_ENCODER_t *rotary_encoder)
{
  if(rotary_encoder->Is_Motor_Object_Ready != NULL)
  {
    rotary_encoder->Flag_Motor_Object_Ready = rotary_encoder->Is_Motor_Object_Ready();
    return rotary_encoder->Flag_Motor_Object_Ready;
  }
  else
  {
    return false;
  }
}

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Send_Data_To_Motor_Thread(ROTARY_ENCODER_t *rotary_encoder)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_ERROR;

  if(ROTARY_ENCODER_Is_Motor_Object_Ready(rotary_encoder) == true)
  {
    taskENTER_CRITICAL();
    osMessagePut(MsgBox, (uint32_t)rotary_encoder->mptr, osWaitForever);  // Send Message
    execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_OK;  
    taskEXIT_CRITICAL();
  }
  else
  {
    if(rotary_encoder->RE_DEBUG.Count_Busy_Motor_Object++ >= 500)
    {
      rotary_encoder->RE_DEBUG.Count_Busy_Motor_Object = 0;
      HAL_UART_Transmit(Debug_UART, "Motor_Object is busy\r\n", sizeof("Motor_Object is busy\r\n"), 10);
    }
    execute_status = ROTARY_ENCODER_STATUS_MOTOR_OBJECT_IS_BUSY; 
  }

  return execute_status;
}
