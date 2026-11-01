/*
 * motor2.c
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */


#include "motor2.h"

extern TIM_HandleTypeDef htim1;   // isti timer kao motor 1 (PE11 = TIM1_CH2)

#define MOTOR2_ARR   2560         // ARR+1 (period u tickovima) = 2560

/* nSLEEP = PA8, DIR = PA10, PWM = PE11 (TIM1_CH2) */

void Motor2_Init(void)
{
//    /* Drajver u sleep dok se ne podesi sve */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   // nSLEEP = 0 (spava)



//    /* 1. Reset drajvera - probudi ga i obrisi fault (toggle nSLEEP) */
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // SLP = 0 (sleep)
//    HAL_Delay(2000);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    // SLP = 1 (wake, brise fault)
//    HAL_Delay(2000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // DIR = 0

    /* Pokreni PWM (za sada 0% dok ne zadamo duty) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Motor2_SetDuty(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;

    uint32_t pulse = (uint32_t)((percent / 100.0f) * MOTOR2_ARR);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
}

void Motor2_SetDir(uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor2_Enable(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor2_Stop(void)
{
    Motor2_SetDuty(0.0f);
}
