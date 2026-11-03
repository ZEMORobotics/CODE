/*
 *   SLP1 = PD11   DIR1 = PD9    PWM1 = PE9  (TIM1_CH1)   -> Motor 1 (left wheel)
 *   SLP2 = PA10   DIR2 = PA8    PWM2 = PE11 (TIM1_CH2)   -> Motor 2 (right wheel)
 *
 *   ENC1: A=PB3 B=PA15 (TIM2)                            -> left wheel encoder
 *   ENC2: A=PC7 B=PC6 (TIM3)                             -> right wheel encoder
 *   SENS1 = PE4 (back IR), SENS2 = PE5 (front IR)    -> main.h
 *
 */

#ifndef LIB_BOARD_CONFIG_H_
#define LIB_BOARD_CONFIG_H_


#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim7, htim2, htim3;

// PID
#define CONTROL_FREQ_HZ      500        /* PID */
#define CONTROL_PERIOD_S     (1.0f / (float)CONTROL_FREQ_HZ)   /* T = 0.002 s   */

#define FEEDBACK_FREQ_HZ     100        /* PID messages to RASPBERRY */

#define TIM7_INPUT_CLK_HZ    64000000UL /* APB1 timer clock (Clock Configuration) */
#define TIM7_PRESCALER       63         /* 64 MHz / 64 = 1 MHz  */

#define PID_DEFAULT_KP            5.7f
#define PID_DEFAULT_KI            0.118f
#define PID_DEFAULT_KD            0.0f
#define PID_D_LPF_FC_HZ           20.0f   /* f_c filter */
#define PID_COMMAND_LIMIT_PCT     100.0f
#define PID_INTEGRATOR_LIMIT_PCT  100.0f

// Motors
#define MOTOR_SUPPLY_VOLTAGE_DEFAULT  34.0f
#define MOTOR_MAX_VOLTAGE             18.0f
#define MOTOR_SUPPLY_VOLTAGE_MIN      14.0f
#define MOTOR_SUPPLY_VOLTAGE_MAX      36.0f
#define MOTOR_ABSOLUTE_MAX_DUTY       64.29f

// Encoders
#define ENC_LEFT_TIMER        htim2     /* ENC1, 32-bit */
#define ENC_RIGHT_TIMER       htim3     /* ENC2, 16-bit */
#define ENCODER_TICKS_PER_REV (2048 * 4)
#define ENC_LEFT_SIGN         (+1)		/*Used for direction calculation, however this is handled in hardware so this yet another claud bs*/
#define ENC_RIGHT_SIGN        (+1)		/*TODO: Test then delete this shit*/
#define RAD_PER_TICK          (6.283185307f / (float)ENCODER_TICKS_PER_REV)


#define SPEED_LPF_FC_HZ       30.0f

#define MAX_WHEEL_SPEED_RADS  60.0f


/* IR: 1 = pin HIGH -> detected */
#define IR_ACTIVE_HIGH    1
#define IR_DEBOUNCE_MS    5


/*
 * Ozicenje koje ovo podrazumeva: kabl kratko spaja pin na GND dok je utaknut.
 *   utaknut cinc  -> pin LOW  -> salje se 0 (jos nije start)
 *   izvucen cinc  -> pin HIGH -> salje se 1 (START)
 * Ako si ozicio obrnuto, promeni CINC_ACTIVE_HIGH na 0.
 *
 * PAZNJA: pri ovakvom ozicenju prekid zice ili ispao konektor cita se kao
 * "izvucen" i robot krece. Proveri kontakt pre meca.
 *
 * Ako pinu das labelu CINC u CubeMX-u, on ce sam napraviti CINC_Pin i
 * CINC_GPIO_Port u main.h - tada obrisi ove dve linije. */
#ifndef CINC_Pin
  #define CINC_Pin        GPIO_PIN_6
  #define CINC_GPIO_Port  GPIOE
#endif

#define CINC_ACTIVE_HIGH  1
#define CINC_DEBOUNCE_MS  20    /* mehanicki kontakt poskakuje - treba vise nego IR */


#endif /* LIB_BOARD_CONFIG_H_ */
