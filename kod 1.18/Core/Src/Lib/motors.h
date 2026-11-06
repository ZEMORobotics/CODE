#ifndef LIB_MOTORS_H_
#define LIB_MOTORS_H_

#include <stdbool.h>
#include <math.h>

#include "ADC.h"
#include "motor.h"

#define MOTOR_SUPPLY_VOLTAGE_DEFAULT  34.0f
#define MOTOR_MAX_VOLTAGE             18.0f
#define MOTOR_SUPPLY_VOLTAGE_MIN      14.0f
#define MOTOR_SUPPLY_VOLTAGE_MAX      36.0f
#define MOTOR_ABSOLUTE_MAX_DUTY       64.29f

void  PWM_Init(void);
void  PWM_Enable(bool on);
void  PWM_Stop(void);

void  PWM_SetCommand_Left(float percent);
void  PWM_SetCommand_Right(float percent);


#endif /* LIB_MOTORS_H_ */
