#include "encoders.h"
#include "main.h"

static uint16_t right_prev_count = 0;
static uint32_t right_total_count = 0;

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

    right_prev_count = 0;
    right_total_count = 0;
}

int32_t encoder_get_count_left_motor(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&ENC_LEFT_TIMER);
}

int32_t encoder_get_count_right_motor(void)
{
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_RIGHT_TIMER);

    int16_t delta = (int16_t)(uint16_t)(current - right_prev_count);

    right_prev_count = current;
    right_total_count += (int32_t)delta;

    return right_total_count;
}
