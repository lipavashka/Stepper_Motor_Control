#include "stepper_motor_state_mashine.h"
#include "execute_stepper_motor.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern volatile MOTOR_Queue_t MOTOR_Queue_RX;

STEPPER_MOTOR_CONTROL_t STEPPER_MOTOR_CONTROL;

void STEPPER_MOTOR_Init_State_Mashine(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  STEPPER_MOTOR_Set_State(MOTOR_STATE_IDLE, MOTOR_STATE_INIT, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_Run_State_Mashine(void)
{
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

  taskENTER_CRITICAL();
  HAL_UART_Transmit(&huart1, "RUN_STEPPER_MOTOR_TASK  ", sizeof("RUN_STEPPER_MOTOR_TASK  "), 10);
  /*stepper_motor->STATE_MASHINE.Current = MOTOR_STATE_INIT;
  switch(stepper_motor->STATE_MASHINE.Current)
  {
    case(MOTOR_STATE_INIT):
    {
      // HAL_UART_Transmit(&huart1, "RUN INIT  ", sizeof("RUN INIT  "), 10);
      STEPPER_MOTOR_RUN_INIT_State(stepper_motor);
      break;  
    }
    case(MOTOR_STATE_IDLE):
    {
      HAL_UART_Transmit(&huart1, "RUN IDLE  ", sizeof("RUN IDLE  "), 10);
      STEPPER_MOTOR_RUN_IDLE_State(stepper_motor);
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
  }*/
  HAL_UART_Transmit(&huart1, "END_STEPPER_MOTOR_TASK\r\n", sizeof("END_STEPPER_MOTOR_TASK\r\n"), 10);
  taskEXIT_CRITICAL();
}

static void STEPPER_MOTOR_Set_State(MOTOR_STATE_t previous, MOTOR_STATE_t current, MOTOR_STATE_t next)
{
  // HAL_UART_Transmit(&huart1, "SET STATE  ", sizeof("SET STATE  "), 10);
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Previous = previous;
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Current = current;
  STEPPER_MOTOR_CONTROL.STATE_MASHINE.Next = next;
  // HAL_UART_Transmit(&huart1, "END SET STATE  ", sizeof("END SET STATE  "), 10);
}

void STEPPER_MOTOR_RUN_IDLE_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  // HAL_UART_Transmit(&huart1, "RUN IDLE  ", sizeof("RUN IDLE  "), 10);
  // STEPPER_MOTOR_Set_State(stepper_motor, stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_INIT, MOTOR_STATE_DEFAULT);
  stepper_motor->STATE_MASHINE.Previous = stepper_motor->STATE_MASHINE.Current;
  stepper_motor->STATE_MASHINE.Current = MOTOR_STATE_INIT;
  // stepper_motor->STATE_MASHINE.Next = MOTOR_STATE_DEFAULT;
  // HAL_UART_Transmit(&huart1, "END IDLE  ", sizeof("END IDLE  "), 10);
}

void STEPPER_MOTOR_RUN_INIT_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  // HAL_UART_Transmit(&huart1, "RUN INIT  ", sizeof("RUN INIT  "), 10);
  STEPPER_MOTOR_Set_State(stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_WAITING_DATA, MOTOR_STATE_DEFAULT);
  // HAL_UART_Transmit(&huart1, "END INIT  ", sizeof("END INIT  "), 10);
}

void STEPPER_MOTOR_RUN_WAITING_DATA_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  STEPPER_MOTOR_Set_State(stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_PARSE_DATA, MOTOR_STATE_DEFAULT);
}

void STEPPER_MOTOR_RUN_PARSE_DATA_State(STEPPER_MOTOR_CONTROL_t *stepper_motor)
{
  STEPPER_MOTOR_Set_State(stepper_motor->STATE_MASHINE.Current, MOTOR_STATE_COMPETE_PARSE_DATA, MOTOR_STATE_DEFAULT);
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
