#ifndef LIB_MOTORS_H_
#define LIB_MOTORS_H_

#include <stdbool.h>
#include <math.h>

#include "board_config.h"
#include "ADC.h"
#include "motor.h"

void  PWM_Init(void);
void  PWM_Enable(bool on);
void  PWM_Stop(void);

void  PWM_SetCommand_Left(float percent);
void  PWM_SetCommand_Right(float percent);


#endif /* LIB_MOTORS_H_ */
