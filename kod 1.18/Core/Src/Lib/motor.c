/*
 * motor.c
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */


#include "motor.h"

extern TIM_HandleTypeDef htim1;   // CubeMX ga pravi

#define MOTOR_ARR   2560          // ARR+1 (period u tickovima) = 2560

/* nSLEEP = PD11, DIR1 = PD9 - proveri da su ti #define iz CubeMX-a bas ovi
   (obicno GPIOD, GPIO_PIN_11 i GPIO_PIN_9). Ako je CubeMX dao labele, koristi njih. */

void Motor_Init(void)
{
//    /* Drajver u sleep dok se ne podesi sve */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);   // nSLEEP = 0 (spava)



//    /* 1. Reset drajvera - probudi ga i obrisi fault (toggle nSLEEP) */
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);  // SLP = 0 (sleep)
//    HAL_Delay(2000);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);    // SLP = 1 (wake, brise fault)
//    HAL_Delay(2000);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_RESET);   // DIR = 0

    /* Pokreni PWM (za sada 0% dok ne zadamo duty) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Motor_SetDuty(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t pulse = (uint32_t)((percent / 100.0f) * MOTOR_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void Motor_SetDir(uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Enable(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Stop(void)
{
    Motor_SetDuty(0.0f);
}
