/*
 * motor.h
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */

#ifndef SRC_LIB_MOTOR_H_
#define SRC_LIB_MOTOR_H_



#include "main.h"

void Motor_Init(void);                 // pozovi jednom u main()
void Motor_SetDuty(float percent);     // 0.0 - 100.0
void Motor_SetDir(uint8_t dir);        // 0 ili 1
void Motor_Enable(uint8_t on);         // 1 = radi, 0 = sleep
void Motor_Stop(void);                 // duty 0

void Motor_TestNoPWM(void);

#endif
