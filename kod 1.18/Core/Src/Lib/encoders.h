#ifndef LIB_ENCODERS_H_
#define LIB_ENCODERS_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim2, htim3;

#define ENC_LEFT_TIMER        htim2     /* ENC1, 32-bit */
#define ENC_RIGHT_TIMER       htim3     /* ENC2, 16-bit */

#define TWO_PI_F 6.283185307179586f
#define ENCODER_TICKS_PER_REV 8192		/* 4 * 2048 */
#define RAD_PER_TICK          (TWO_PI_F / (float)ENCODER_TICKS_PER_REV)

#define ENC_LEFT_SIGN         (+1)		/*Used for direction calculation, however this is handled in hardware so this yet another claud bs*/
#define ENC_RIGHT_SIGN        (+1)		/*TODO: Test then delete this shit*/

void    encoder_init(void);
void    encoder_reset(void);
int32_t encoder_get_count_left_motor(void);
int32_t encoder_get_count_right_motor(void);

#endif /* LIB_ENCODERS_H_ */
