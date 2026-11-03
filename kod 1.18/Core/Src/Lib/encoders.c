#include "encoders.h"
#include "main.h"

void encoder_init(void)
{
	if(HAL_TIM_Encoder_Start(&ENC_LEFT_TIMER, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();

	if(HAL_TIM_Encoder_Start(&ENC_RIGHT_TIMER, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();

    encoder_reset();
}

void encoder_reset(void)
{
    __HAL_TIM_SET_COUNTER(&ENC_LEFT_TIMER, 0);
    __HAL_TIM_SET_COUNTER(&ENC_RIGHT_TIMER, 0);
}

int32_t encoder_get_count_left_motor(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&ENC_LEFT_TIMER);
}

int32_t encoder_get_count_right_motor(void)
{
	return (int32_t)__HAL_TIM_GET_COUNTER(&ENC_RIGHT_TIMER);
}
