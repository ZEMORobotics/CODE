/*
 * motor.h
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 *
 *      Updated on: Nov 3, 2026
 * 		Author: Vladimirko pipiriko
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

void Motor_Init(void);                 // pozovi jednom u main()
void Motor_SetDuty(float percent);     // 0.0 - 100.0
void Motor_SetDir(uint8_t dir);        // 0 ili 1
void Motor_Enable(uint8_t on);         // 1 = radi, 0 = sleep
void Motor_Stop(void);                 // duty 0

#endif
