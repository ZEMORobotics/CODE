/*
 * motor.c
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */
#include "motor.h"

// PWM, same timer, channel 1
extern TIM_HandleTypeDef htim1;

static uint32_t motor_period = 0;

void Motor_Init(void)
{
//    /* Drajver u sleep dok se ne podesi sve */
    HAL_GPIO_WritePin(SLP1_GPIO_Port, SLP1_Pin, GPIO_PIN_RESET);   // nSLEEP = 0 (spava)


    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin,  GPIO_PIN_RESET);   // DIR = 0

    motor_period =  __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Motor_SetDuty(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t pulse = (uint32_t)((percent / 100.0f) * motor_period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void Motor_SetDir(uint8_t dir)
{
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Enable(uint8_t on)
{
    HAL_GPIO_WritePin(SLP1_GPIO_Port, SLP1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Stop(void)
{
    Motor_SetDuty(0.0f);
}
