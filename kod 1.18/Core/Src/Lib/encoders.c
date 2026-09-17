#include "encoders.h"

void encoder_init(void)
{
	HAL_TIM_Encoder_Start(&ENC_LEFT_TIMER, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENC_RIGHT_TIMER, TIM_CHANNEL_ALL);
	encoder_reset();
}

void encoder_reset(void)
{
	ENC_LEFT_TIMER.Instance->CNT = 0;
	ENC_RIGHT_TIMER.Instance->CNT = 0;
}

int32_t encoder_get_count_left_motor(void)
{
	return (int32_t)ENC_LEFT_TIMER.Instance->CNT;
}

int32_t encoder_get_count_right_motor(void)
{
	return (int32_t)ENC_RIGHT_TIMER.Instance->CNT;
}
