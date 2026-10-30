/*
 * encoders.h
 */
#ifndef LIB_ENCODERS_H_
#define LIB_ENCODERS_H_
#include "board_config.h"
#include <stdint.h>

void    encoder_init(void);
void    encoder_reset(void);
int32_t encoder_get_count_left_motor(void);
int32_t encoder_get_count_right_motor(void);

#endif /* LIB_ENCODERS_H_ */
