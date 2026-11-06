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

// Update rates
#define CONTROL_FREQ_HZ      500        /* PID */
#define CONTROL_PERIOD_S     (1.0f / (float)CONTROL_FREQ_HZ)   /* T = 0.002 s   */

#define FEEDBACK_FREQ_HZ     100        /* PID messages to RASPBERRY */

/* IR: 1 = pin HIGH -> detected */
#define IR_ACTIVE_HIGH    1
#define IR_DEBOUNCE_MS    5


#ifndef CINC_Pin
  #define CINC_Pin        GPIO_PIN_6
  #define CINC_GPIO_Port  GPIOE
#endif

#define CINC_ACTIVE_HIGH  1
#define CINC_DEBOUNCE_MS  50

#endif /* LIB_BOARD_CONFIG_H_ */
