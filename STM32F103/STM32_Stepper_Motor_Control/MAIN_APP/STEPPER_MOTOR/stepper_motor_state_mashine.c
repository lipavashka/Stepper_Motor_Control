#include "stepper_motor_state_mashine.h"
#include "execute_stepper_motor.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern volatile MOTOR_Queue_t MOTOR_Queue_RX;
// extern osPoolId  mpool;
extern osMessageQId  MsgBox;
MOTOR_Queue_t  *rx_rptr;

extern osEvent  rx_evt;
uint32_t current_state = 0;

STEPPER_MOTOR_CONTROL_t STEPPER_MOTOR_CONTROL;

void STEPPER_MOTOR_Init_State_Mashine(void)
{
  STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process = false;
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
  // Execute_Motor_Waiting_Data(&STEPPER_MOTOR_CONTROL);
  // STEPPER_MOTOR_RUN_WAITING_DATA_State();
  
//  rx_evt = osMessageGet(MsgBox, osWaitForever);  // wait for message
//  if (rx_evt.status == osEventMessage) 
//  {
//    taskENTER_CRITICAL();
//    if(rx_evt.value.p != NULL)
//    {
//      rx_rptr = rx_evt.value.p;
//
//      MOTOR_Queue_RX.Enable_0 = rx_rptr->Enable_0;
//      MOTOR_Queue_RX.Enable_1 = rx_rptr->Enable_1;
//      MOTOR_Queue_RX.Direction_0 = rx_rptr->Direction_0;
//      MOTOR_Queue_RX.Direction_1 = rx_rptr->Direction_1;
//      MOTOR_Queue_RX.Period_0 = rx_rptr->Period_0;
//      MOTOR_Queue_RX.Period_1 = rx_rptr->Period_1;
//      MOTOR_Queue_RX.DutyCycle_0 = rx_rptr->DutyCycle_0;
//      MOTOR_Queue_RX.DutyCycle_1 = rx_rptr->DutyCycle_1;
//      MOTOR_Queue_RX.counter = rx_rptr->counter;
//
//      // osPoolFree(mpool, rx_rptr);                  // free memory allocated for message
//      /*HAL_UART_Transmit(&huart3, "Queue Motor RX <<  ", sizeof("Queue Motor RX <<  "), 10);      
//      // <- here show message
//      HAL_UART_Transmit(&huart3, "\r\n", 2, 10);*/
//    }
//    taskEXIT_CRITICAL();
//
//     /* taskENTER_CRITICAL();
//    if(MOTOR_Queue_RX.Enable_0 == true)
//    {
//      setPeriod(MOTOR_Queue_RX.Period_0); // setPeriod(period_value);
//      // set_PWM_0(MOTOR_Queue_RX.DutyCycle_0);
//      SET_PWM_0(MOTOR_Queue_RX.DutyCycle_0);
//      Set_Motor_Direction_0(MOTOR_Queue_RX.Direction_0);
//      Enable_Motor_0();
//    }
//    else
//    {
//      Disable_Motor_0();
//    }
//    taskEXIT_CRITICAL();*/
//  }
  
  /*taskENTER_CRITICAL();
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
  taskEXIT_CRITICAL();*/

 /*   taskENTER_CRITICAL();
  HAL_UART_Transmit(&huart1, "RUN_STEPPER_MOTOR_TASK  ", sizeof("RUN_STEPPER_MOTOR_TASK  "), 10);
 // STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current = MOTOR_STATE_INIT;
 if (current_state == 0)
  {
    current_state = 1;
    HAL_UART_Transmit(&huart1, "CURRENT STATE 0  ", sizeof("CURRENT STATE 0  "), 10);
  }
  else if (current_state == 1)
  {
    current_state = 2;
    HAL_UART_Transmit(&huart1, "CURRENT STATE 1  ", sizeof("CURRENT STATE 1  "), 10);
  }
  else
  {
    current_state = 0;
    HAL_UART_Transmit(&huart1, "CURRENT STATE Don't know  ", sizeof("CURRENT STATE Don't know  "), 10);
  }*/
/*  switch(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current)
  {    
    case(MOTOR_STATE_INIT):
    {
      HAL_UART_Transmit(&huart1, "RUN INIT  ", sizeof("RUN INIT  "), 10);
      STEPPER_MOTOR_RUN_INIT_State();
      break;  
    }
    case(MOTOR_STATE_IDLE):
    {
      HAL_UART_Transmit(&huart1, "RUN IDLE  ", sizeof("RUN IDLE  "), 10);
      STEPPER_MOTOR_RUN_IDLE_State();
      break;  
    }
    case(MOTOR_STATE_WAITING_DATA):
    {
      STEPPER_MOTOR_RUN_WAITING_DATA_State(stepper_motor);
      break;  
    }
    case(MOTOR_STATE_PARSE_DATA):
    {
      // STEPPER_MOTOR_RUN_PARSE_DATA_State(stepper_motor);
      break;  
    }
    case(MOTOR_STATE_COMPETE_PARSE_DATA):
    {
      // STEPPER_MOTOR_RUN_COMPLETE_PARSE_DATA_State(stepper_motor);
      break;  
    }    
    case(MOTOR_STATE_DEFAULT):
    {
      // STEPPER_MOTOR_RUN_DEFAULT_State(stepper_motor);
      break;  
    }
    case(MOTOR_STATE_ERROR):
    {
      // STEPPER_MOTOR_RUN_ERROR_State(stepper_motor);
      break;  
    }
    default:
    {
      break;
    }       
  } 
  HAL_UART_Transmit(&huart1, "END_STEPPER_MOTOR_TASK\r\n", sizeof("END_STEPPER_MOTOR_TASK\r\n"), 10);
  taskEXIT_CRITICAL();*/
}

static void STEPPER_MOTOR_Set_State(MOTOR_STATE_t previous, MOTOR_STATE_t current, MOTOR_STATE_t next)
{
  // HAL_UART_Transmit(&huart1, "SET STATE  ", sizeof("SET STATE  "), 10);
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Previous = previous;
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current = current;
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Next = next;
  // HAL_UART_Transmit(&huart1, "END SET STATE  ", sizeof("END SET STATE  "), 10);
}

void STEPPER_MOTOR_RUN_IDLE_State(void)
{
  STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_INIT, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_INIT_State(void)
{
  HAL_UART_Transmit(&huart1, "RUN INIT\r\n", sizeof("RUN INIT\r\n"), 10);
  STEPPER_MOTOR_CONTROL.Flag_Complete_Motor_Process = false;
  STEPPER_MOTOR_Set_State(STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_WAITING_DATA_State(void)
{
  MOTOR_STATUS_t execute_status = MOTOR_STATUS_WAITING_DATA_ERROR;
  
  HAL_UART_Transmit(&huart1, "RUN WAITING_DATA\r\n", sizeof("RUN WAITING_DATA\r\n"), 10);
  
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
  
  HAL_UART_Transmit(&huart1, "RUN PARSE\r\n", sizeof("RUN PARSE\r\n"), 10);

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

void STEPPER_MOTOR_RUN_COMPLETE_PARSE_DATA_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  STEPPER_MOTOR_Set_State(stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_DEFAULT_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  STEPPER_MOTOR_Set_State(stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_INIT, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_ERROR_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  STEPPER_MOTOR_Set_State(stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_INIT, MOTOR_STATE_DEFAULT);
}
