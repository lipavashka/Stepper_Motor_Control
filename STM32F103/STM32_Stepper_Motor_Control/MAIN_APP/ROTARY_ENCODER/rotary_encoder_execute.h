#pragma once

#include "cmsis_os.h"
#include "stdint.h"
#include "stdbool.h"
#include "rotary_encoder_type.h"

ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Waiting_Data();
ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Parse_Data(ROTARY_ENCODER_t *rotary_encoder);
ROTARY_ENCODER_STATUS_t Execute_RotaryEncoder_Send_Data_To_Motor_Thread(ROTARY_ENCODER_t *rotary_encoder);
