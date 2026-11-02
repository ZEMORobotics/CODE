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
#include "tim.h"

void Motor_Init(void);

void Motor_SetDuty(float percent);
void Motor_SetDir(uint8_t dir);        // 0 or 1
void Motor_Enable(uint8_t on);         // 0 = sleep
void Motor_Stop(void);                 // duty 0

void Motor2_SetDuty(float percent);
void Motor2_SetDir(uint8_t dir);        // 0 or 1
void Motor2_Enable(uint8_t on);         // 0 = sleep
void Motor2_Stop(void);                 // duty 0

#endif
