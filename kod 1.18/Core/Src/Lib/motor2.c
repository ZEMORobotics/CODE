/*
 * motor2.c
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */


#include "motor2.h"

// PWM, same timer, channel 2
extern TIM_HandleTypeDef htim1;

static uint32_t motor2_period = 0;

void Motor2_Init(void)
{
    HAL_GPIO_WritePin(SLP2_GPIO_Port, SLP2_Pin, GPIO_PIN_RESET);   // sleep mode
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);  // DIR = 0

    motor2_period =  __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Motor2_SetDuty(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t pulse = (uint32_t)((percent / 100.0f) * motor2_period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

void Motor2_SetDir(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor2_Enable(uint8_t on)
{
    HAL_GPIO_WritePin(SLP2_GPIO_Port, SLP2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor2_Stop(void)
{
    Motor2_SetDuty(0.0f);
}
