#include "stepper_motor_state_mashine.h"
#include "execute_stepper_motor.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef *Debug_UART;
extern osMessageQId  MsgBox;

STEPPER_MOTOR_CONTROL_t STEPPER_MOTOR_CONTROL;

void STEPPER_MOTOR_Init_State_Mashine(void)
{
  STEPPER_MOTOR_CONTROL.Flag_Object_Ready = false;
  STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process = false;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.Enable_0 = false;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.Enable_1 = false;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.Period_0 = 0;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.Period_1 = 0;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.DutyCycle_0 = 0;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.DutyCycle_1 = 0;
  STEPPER_MOTOR_CONTROL.MOTOR_Queue_RX.counter = 0;

  STEPPER_MOTOR_Set_State(MOTOR_STATE_IDLE, MOTOR_STATE_IDLE, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_Run_State_Mashine(void)
{
  do
  {
    switch(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current)
    {
      case(MOTOR_STATE_IDLE):
      {
        STEPPER_MOTOR_RUN_IDLE_State();
        break;  
     }
      case(MOTOR_STATE_INIT):
      {
        STEPPER_MOTOR_RUN_INIT_State();
        break;  
      }
      case(MOTOR_STATE_WAITING_DATA):
      {
        STEPPER_MOTOR_RUN_WAITING_DATA_State();
        break;  
      }
      case(MOTOR_STATE_PARSE_DATA):
      {
        STEPPER_MOTOR_RUN_PARSE_DATA_State();
        break;  
      }
      default:
        break;
    }
  }while(STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process == false);
  STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process = false;
}

static void STEPPER_MOTOR_Set_State(MOTOR_STATE_t previous, MOTOR_STATE_t current, MOTOR_STATE_t next)
{
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Previous = previous;
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current = current;
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Next = next;
}

void STEPPER_MOTOR_RUN_IDLE_State(void)
{
  STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_INIT, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_INIT_State(void)
{
  HAL_UART_Transmit(Debug_UART, "RUN INIT\r\n", sizeof("RUN INIT\r\n"), 10);
  STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process = false;
  STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_WAITING_DATA_State(void)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  
  HAL_UART_Transmit(Debug_UART, "RUN WAITING_DATA\r\n", sizeof("RUN WAITING_DATA\r\n"), 10);
  
  execute_status = Execute_Motor_Waiting_Data(&STEPPER_MOTOR_CONTROL);

  switch(execute_status)
  {
    case(MOTOR_STATUS_WAITING_DATA_OK):
    {
      STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_PARSE_DATA, MOTOR_STATE_DEFAULT);      
      break;
    }
    case(MOTOR_STATUS_WAITING_DATA_ERROR):
    {
      STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
      break;
    }
    default:
      STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
      break;
  }
}

void STEPPER_MOTOR_RUN_PARSE_DATA_State(void)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  
  HAL_UART_Transmit(Debug_UART, "RUN PARSE\r\n", sizeof("RUN PARSE\r\n"), 10);

  execute_status = Execute_Motor_Parse_Data(&STEPPER_MOTOR_CONTROL);

  switch(execute_status)
  {
    case(MOTOR_STATUS_PARSE_DATA_OK):
    {
      STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);     
      break;
    }
    case(MOTOR_STATUS_PARSE_DATA_ERROR):
    {
      STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
      break;
    }
    default:
      STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
      break;
  }
  
  STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process = true;
}

bool STEPPER_MOTOR_Get_Motor_Object_Ready_Flag(void)
{
  return STEPPER_MOTOR_CONTROL.Flag_Object_Ready;
}
