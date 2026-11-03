/*
 * odometry.h - enkoderi -> brzina tocka [rad/s] + ukupni tickovi.
 *
 * Na STM-u se od "odometrije" koristi samo ovo:
 *   - razlika brojaca po tiku (500 Hz), sa ispravnim prelaskom preko nule brojaca
 *   - ukupni tickovi od reseta (int32, za CAN WHEEL_POSITION = 102)
 *   - ugaona brzina tocka [rad/s], sirova i filtrirana (za PID i CAN WHEEL_FEEDBACK_VEL = 101)
 *
 * Poziciju robota (x, y, ugao) racuna ros2. Kod za to je ostao ispod
 * ODOMETRY_ON_STM (board_config.h) i po defaultu se NE kompajlira.
 */
#ifndef LIB_ODOMETRY_H_
#define LIB_ODOMETRY_H_

#include "board_config.h"
#include "encoders.h"
#include "lowpass.h"
#include <stdint.h>


void odometry_init(void);      /* posle encoder_init(); racuna koeficijent filtera */
void odometry_reset(void);     /* nula brojace, brzine i filter (i pozu ako je ukljucena) */

// TIM7
void odometry_update(void);

float get_wheel_speed_left(void);
float get_wheel_speed_right(void);
float get_wheel_speed_left_raw(void);
float get_wheel_speed_right_raw(void);

int32_t get_encoder_total_left(void);   /* ukupni tickovi od reseta (CAN 102) */
int32_t get_encoder_total_right(void);


#endif /* LIB_ODOMETRY_H_ */
