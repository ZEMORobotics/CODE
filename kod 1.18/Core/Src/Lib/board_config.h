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

// Motors
#define MOTOR_SUPPLY_VOLTAGE_DEFAULT  34.0f
#define MOTOR_MAX_VOLTAGE             18.0f
#define MOTOR_SUPPLY_VOLTAGE_MIN      14.0f
#define MOTOR_SUPPLY_VOLTAGE_MAX      36.0f
#define MOTOR_ABSOLUTE_MAX_DUTY       64.29f

// Encoders
#define ENC_LEFT_TIMER        htim2     /* ENC1, 32-bitni brojac */
#define ENC_RIGHT_TIMER       htim3     /* ENC2, 16-bitni brojac */
#define ENCODER_TICKS_PER_REV (2048 * 4) /* TI12 mod = 4x od 2048 CPR */
#define ENC_LEFT_SIGN         (+1)      /* +1 ako brojac raste kad tocak ide napred, inace -1 */
#define ENC_RIGHT_SIGN        (+1)
#define RAD_PER_TICK          (6.283185307f / (float)ENCODER_TICKS_PER_REV)


/* Brzina iz jednog tika (2 ms) ima kvantizacioni sum od +-1 tick, sto je
 * +-0.38 rad/s. Prvi red niskopropusnog filtera (lowpass.h)
 * :
 *      filtered = a * filtrirano_staro + (1 - a) * raw
 *      a = e^(-2*pi * f_c * T),  T = CONTROL_PERIOD_S
 */
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

/* PID regulator brzine tocka (PID.c) --------------------------------------
 * Pozicioni PID, isti za oba tocka (jedan zajednicki skup koeficijenata):
 *
 *   error       = reference - feedback                       [rad/s]
 *   P           = Kp * error                                  [%]
 *   I          += Ki * error       (klesten, anti-windup)     [%]
 *   D_raw       = error - error_previous                      (po tiku)
 *   D_filtered  = a * D_filtered + (1 - a) * D_raw,  a = e^(-2*pi*fc*T)
 *   command     = P + I + Kd * D_filtered   (klesteno na +-100 %)
 *
 * Jedinice: ulaz u rad/s, izlaz u % maksimalnog PWM-a. Ki i Kd su "po tiku"
 * (T = 2 ms je vec uracunat u njih), kao i u referentnom PIC32 kodu.
 *
 * Podrazumevane vrednosti daju ISTI odziv kao stari regulator u mm/s
 * (Kp 2.8, Ki 0.058 na 2000 mm/s): 2.8 * 40.77 / 20 = 5.7, 0.058 * 40.77 / 20 = 0.118.
 * Sve tri se menjaju preko CAN-a (PID_KP, PID_KI, PID_KD = 103, 104, 105). */
#define PID_DEFAULT_KP            5.7f
#define PID_DEFAULT_KI            0.118f
#define PID_DEFAULT_KD            0.0f
#define PID_D_LPF_FC_HZ           20.0f   /* fc filtera D clana */
#define PID_COMMAND_LIMIT_PCT     100.0f  /* izlaz regulatora, +- % maks. PWM-a */
#define PID_INTEGRATOR_LIMIT_PCT  100.0f  /* klestenje I clana, +- %             */

/* Odometrija na STM-u - ISKLJUCENA, radi je ros2 ------------------------- */
/* Kod za x, y i ugao je ostao u odometry.c ali se ne kompajlira. Ako ikad
 * zatreba, stavi 1 i ove dimenzije postaju vazne. */
#ifndef ODOMETRY_ON_STM
  #define ODOMETRY_ON_STM 0
#endif
#if ODOMETRY_ON_STM
  #define WHEEL_DIAMETER_MM     81.54f
  #define WHEEL_RADIUS_MM       (WHEEL_DIAMETER_MM / 2.0f)
  #define WHEEL_BASE_MM         422.0f
  #define RADS_TO_MMS(w)  ((w) * WHEEL_RADIUS_MM)
  #define MMS_TO_RADS(v)  ((v) / WHEEL_RADIUS_MM)
#endif


#endif /* LIB_BOARD_CONFIG_H_ */
