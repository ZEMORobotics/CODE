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




/* TEST: reverse preko truth tabele, EN cvrsto na 1 (bez PWM).
   nSLEEP=1, EN=1, PH=0 -> reverse drive.
   PAZNJA: pun napon reverse (bez PWM ogranicenja)! Kratko / bez motora! */
void Motor2_TestReverse(void)
{
    /* PE11 (EN) kao obican GPIO izlaz, ne PWM */
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_11;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);

    /* Probudi drajver */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   // SLP = 0
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);     // SLP = 1 (wake)
    HAL_Delay(5);

    /* Postavi PH = 0 (reverse) PRE nego sto ukljucis EN */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // PH = 0
    HAL_Delay(2);

    /* EN = 1 -> reverse drive (pun napon!) */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);    // EN = 1
}


void Motor2_TestForward(void)  // isto, samo PH=1
{
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);    // PH = 1 (forward)
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);    // EN = 1
}


void Motor2_TestNoPWM(void)
{
    /* Oba ulaza drajvera cvrsto na 0 (coast - svi FET off) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // PH  (PA10) = 0
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);  // EN  (PE11) = 0

    /* Tek sad probudi drajver */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   // SLP = 0
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);     // SLP = 1 (wake)
    HAL_Delay(5);

    /* Ostani ovako - gledaj fault lampicu */
}
