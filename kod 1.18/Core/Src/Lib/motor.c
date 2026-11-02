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
    HAL_GPIO_WritePin(SLP1_GPIO_Port, SLP1_Pin, GPIO_PIN_RESET);   	// Sleep Mode = 0
    HAL_GPIO_WritePin(SLP2_GPIO_Port, SLP2_Pin, GPIO_PIN_RESET);   // Sleep mode = 0

    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin,  GPIO_PIN_RESET);   // DIR = 0
	HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);  // DIR = 0

    motor_period =  __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Motor_SetDuty(float percent)
{
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

void Motor2_SetDuty(float percent)
{
    uint32_t pulse = (uint32_t)((percent / 100.0f) * motor_period);
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
