#ifndef LIB_ODOMETRY_H_
#define LIB_ODOMETRY_H_

#include "board_config.h"
#include "encoders.h"
#include "lowpass.h"
#include <stdint.h>


void odometry_init(void);
void odometry_reset(void);

// TIM7
void odometry_update(void);

float get_wheel_speed_left(void);
float get_wheel_speed_right(void);
float get_wheel_speed_left_raw(void);
float get_wheel_speed_right_raw(void);

int32_t get_encoder_total_left(void);
int32_t get_encoder_total_right(void);


#endif /* LIB_ODOMETRY_H_ */
