#include "rotary_encoder_state_mashine.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

#define R_motor   25u
#define R_encoder 25u
#define _2PI ((2.0) * (PI))

ROTARY_ENCODER_t ROTARY_ENCODER = 
{
   ROTARY_ENCODER_STATUS_ERROR,                 // ROTARY_ENCODER_STATUS_t Status;
   NULL,                                        // QUEUE_MOTOR_t *mptr;
   {
    // QUEUE_MOTOR_t queue_motor_set_parametrs;
    false,                                      // bool Enable_0;
    false,                                      // bool Enable_1;
    QUEUE_MOTOR_DIRECTION_Forward,              // QUEUE_MOTOR_DIRECTION_t Direction_0;
    QUEUE_MOTOR_DIRECTION_Forward,              // QUEUE_MOTOR_DIRECTION_t Direction_1;
    2000,                                       // uint16_t Period_0;
    2000,                                       // uint16_t Period_1;
    3,                                          // uint16_t DutyCycle_0;
    3,                                          // uint16_t DutyCycle_1;
    0                                           // uint32_t counter;  
   },
   0,                                           // uint32_t Read_RAW_Encoder_Value;
   0,                                           // uint32_t RAW_Value;
   32768,                                       // uint32_t RAW_Value_0;
   32768,                                       // uint32_t RAW_Value_1;
   32768,                                       // uint32_t RAW_Value_2;
   {
    // ROTARY_ENCODER_CONVERTER_t CONVERTER;
    R_motor,                                    // const uint32_t Rm;
    (_2PI * R_motor),                     // const double Cm;
    1,                                          // const uint32_t Hols;
    (_2PI * R_motor), // - for example    // uint32_t Cm_SET;  
    R_encoder,                                  // const uint32_t Re;
    (_2PI * R_encoder),                   // const double Ce;  
    10000,                                       // const uint32_t k_Timer;
    0,                                          // uint32_t k__Set;
    0,                                          // uint32_t k;  
    0,                                          // uint32_t number_of_Pulses;
    0                                           // uint16_t Timer_Period_SET;
   },
   false,                                       // bool Flag_Motor_Object_Ready;
   NULL,                                        // Check_Is_Motor_Object_Ready Is_Motor_Object_Ready;
   false,                                       // bool Flag_Complete_Encoder_Process;
   ROTARY_ENCODER_STATE_IDLE                    // ROTARY_ENCODER_STATE_MASHINE_t STATE_MASHINE;
};

void ROTARY_ENCODER_Init_State_Mashine(void)
{
  ROTARY_ENCODER.Flag_Motor_Object_Ready = false;
  ROTARY_ENCODER.Flag_Complete_Encoder_Process = false;
  ROTARY_ENCODER.queue_motor_set_parametrs.Enable_0 = false;
  ROTARY_ENCODER.queue_motor_set_parametrs.Enable_1 = false;
  ROTARY_ENCODER.queue_motor_set_parametrs.Direction_0 = QUEUE_MOTOR_DIRECTION_Forward;
  ROTARY_ENCODER.queue_motor_set_parametrs.Direction_1 = QUEUE_MOTOR_DIRECTION_Forward;
  ROTARY_ENCODER.queue_motor_set_parametrs.Period_0 = 2000;
  ROTARY_ENCODER.queue_motor_set_parametrs.Period_1 = 2000;
  ROTARY_ENCODER.queue_motor_set_parametrs.DutyCycle_0 = 3;
  ROTARY_ENCODER.queue_motor_set_parametrs.DutyCycle_1 = 3;
  ROTARY_ENCODER.queue_motor_set_parametrs.counter = 0;
  ROTARY_ENCODER.mptr = &ROTARY_ENCODER.queue_motor_set_parametrs;

  ROTARY_ENCODER_Set_State(ROTARY_ENCODER_STATE_IDLE, ROTARY_ENCODER_STATE_IDLE, ROTARY_ENCODER_STATE_DEFAULT);
}

void ROTARY_ENCODER_Run_State_Mashine(void)
{
  do
  {
    switch(ROTARY_ENCODER.STATE_MASHINE.Current)
    {
      case(ROTARY_ENCODER_STATE_IDLE):
      {
        ROTARY_ENCODER_RUN_IDLE_State();
        break;  
      }
      case(ROTARY_ENCODER_STATE_INIT):
      {
        ROTARY_ENCODER_RUN_INIT_State();
        break;  
      }
      case(ROTARY_ENCODER_STATE_WAITING_ENCODER_SENSOR_DATA):
      {
        ROTARY_ENCODER_RUN_WAITING_ENCODER_SENSOR_DATA_State();
        break;  
      }
      case(ROTARY_ENCODER_STATE_PARSE_ENCODER_SENSOR_DATA):
      {
        ROTARY_ENCODER_RUN_PARSE_ENCODER_SENSOR_DATA_State();
        break;  
      }
      case(ROTARY_ENCODER_STATE_SEND_DATA_TO_MOTOR_THREAD):
      {
        ROTARY_ENCODER_RUN_SEND_DATA_TO_MOTOR_THREAD_State();
        break;  
      }
      default:
        break;
    }
  }while(ROTARY_ENCODER.Flag_Complete_Encoder_Process == false);
  ROTARY_ENCODER.Flag_Complete_Encoder_Process = false;
}

void ROTARY_ENCODER_RUN_IDLE_State(void)
{
  HAL_UART_Transmit(&huart1, "1.1 RUN IDLE\r\n", sizeof("1.1 RUN IDLE\r\n"), 10);
  ROTARY_ENCODER_Set_State(ROTARY_ENCODER_STATE_IDLE, ROTARY_ENCODER_STATE_INIT, ROTARY_ENCODER_STATE_DEFAULT);
}

void ROTARY_ENCODER_RUN_INIT_State(void)
{
  HAL_UART_Transmit(&huart1, "1.2 RUN INIT\r\n", sizeof("1.2 RUN INIT\r\n"), 10);
  ROTARY_ENCODER.Flag_Complete_Encoder_Process = false;
  ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_WAITING_ENCODER_SENSOR_DATA, ROTARY_ENCODER_STATE_DEFAULT);
}

void ROTARY_ENCODER_RUN_WAITING_ENCODER_SENSOR_DATA_State(void)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_ERROR;

  HAL_UART_Transmit(&huart1, "1.3 RUN WAITING ENCODER SENSOR DATA\r\n", sizeof("1.3 RUN WAITING ENCODER SENSOR DATA\r\n"), 10);

  execute_status = Execute_RotaryEncoder_Waiting_Data(&ROTARY_ENCODER);

  switch(execute_status)
  {
    case(ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_OK):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_PARSE_ENCODER_SENSOR_DATA, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }
    case(ROTARY_ENCODER_STATUS_WAITING_ENCODER_SENSOR_DATA_ERROR):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_PARSE_ENCODER_SENSOR_DATA, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }
    default:
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_PARSE_ENCODER_SENSOR_DATA, ROTARY_ENCODER_STATE_DEFAULT);
      break;
  }
}

void ROTARY_ENCODER_RUN_PARSE_ENCODER_SENSOR_DATA_State(void)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_ERROR;

  HAL_UART_Transmit(&huart1, "1.4 RUN PARSE ENCODER SENSOR DATA\r\n", sizeof("1.4 RUN PARSE ENCODER SENSOR DATA\r\n"), 10);

  execute_status = Execute_RotaryEncoder_Parse_Data(&ROTARY_ENCODER);

  switch(execute_status)
  {
    case(ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_OK):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_SEND_DATA_TO_MOTOR_THREAD, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }
    case(ROTARY_ENCODER_STATUS_PARSE_ENCODER_SENSOR_DATA_ERROR):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_SEND_DATA_TO_MOTOR_THREAD, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }
    default:
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_SEND_DATA_TO_MOTOR_THREAD, ROTARY_ENCODER_STATE_DEFAULT);
      break;
  }
}

void ROTARY_ENCODER_RUN_SEND_DATA_TO_MOTOR_THREAD_State(void)
{
  ROTARY_ENCODER_STATUS_t execute_status = ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_ERROR;

  HAL_UART_Transmit(&huart1, "1.5 RUN SEND DATA TO MOTOR THREAD\r\n", sizeof("1.5 RUN SEND DATA TO MOTOR THREAD\r\n"), 10);

  execute_status = Execute_RotaryEncoder_Send_Data_To_Motor_Thread(&ROTARY_ENCODER);

  switch(execute_status)
  {
    case(ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_OK):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_IDLE, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }
    case(ROTARY_ENCODER_STATUS_MOTOR_OBJECT_IS_BUSY):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Previous, ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }    
    case(ROTARY_ENCODER_STATUS_SEND_DATA_TO_MOTOR_THREAD_ERROR):
    {
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_IDLE, ROTARY_ENCODER_STATE_DEFAULT);
      break;
    }
    default:
      ROTARY_ENCODER_Set_State(ROTARY_ENCODER.STATE_MASHINE.Current, ROTARY_ENCODER_STATE_IDLE, ROTARY_ENCODER_STATE_DEFAULT);
      break;
  }
  ROTARY_ENCODER.Flag_Complete_Encoder_Process = true;
}

static void ROTARY_ENCODER_Set_State(ROTARY_ENCODER_STATE_t previous, ROTARY_ENCODER_STATE_t current, ROTARY_ENCODER_STATE_t next)
{
  ROTARY_ENCODER.STATE_MASHINE.Previous = previous;
  ROTARY_ENCODER.STATE_MASHINE.Current = current;
  ROTARY_ENCODER.STATE_MASHINE.Next = next;
}

void ROTARY_ENCODER_Init_Reference_Motor_Object_Ready(bool (*p_Motor_Object_Ready_Flag)(void))
{
  ROTARY_ENCODER.Is_Motor_Object_Ready = NULL;
  if(p_Motor_Object_Ready_Flag != NULL)
  {
    ROTARY_ENCODER.Is_Motor_Object_Ready = p_Motor_Object_Ready_Flag;
  }
}
