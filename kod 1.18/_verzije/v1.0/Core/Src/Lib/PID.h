/*
 * PID.h - regulator brzine tocka, po jedan za levi i desni tocak.
 *
 * Pozicioni PID sa filtriranim D clanom i anti-windup-om, po uzoru na
 * referentni PIC32 kod (pid_regulator.c):
 *
 *   error       = reference - feedback                        [rad/s]
 *   P           = Kp * error                                   [%]
 *   I          += Ki * error        (klesten na +-integrator_limit)
 *   D_raw       = error - error_previous                       (po tiku)
 *   D_filtered  = a * D_filtered + (1 - a) * D_raw,   a = e^(-2*pi*fc*T)
 *   command     = P + I + Kd * D_filtered   (klesteno na +-command_limit)
 *
 * Anti-windup: ako je izlaz zasicen a greska gura u istom smeru, I clan se
 * ne menja u tom tiku.
 *
 * Jedinice:
 *   reference, feedback, error   rad/s   (float - brzina tocka nije ceo broj)
 *   command                      % maksimalnog PWM-a, -100..+100 (float)
 *   Kp  [% / (rad/s)]     Ki  [% / (rad/s) po tiku]     Kd  [% / (rad/s po tiku)]
 *
 * Regulator nigde ne vidi CCR registar ni napon - to radi motors.c, koji
 * command u % prevede u CCR (100 % = maksimalni dozvoljeni CCR).
 *
 * Oba tocka koriste JEDAN zajednicki skup koeficijenata (pid_gains_t) - na
 * njega pokazuje gains u svakom regulatoru. CAN menja samo taj jedan skup.
 *
 * Sve sto dira i TIM7 prekid i glavna petlja / CAN prekid je volatile.
 */
#ifndef LIB_PID_H_
#define LIB_PID_H_
#include <stdbool.h>

/* Zajednicki koeficijenti za oba tocka. Menjaju se preko CAN-a (PID_KP=103, PID_KI=104, PID_KD=105). */
typedef struct {
    volatile float proportional_gain;              /* Kp */
    volatile float integral_gain;                  /* Ki, po tiku */
    volatile float derivative_gain;                /* Kd, po tiku */
    volatile float derivative_filter_coefficient;  /* a = e^(-2*pi*fc*T), 0..1 */
} pid_gains_t;

typedef struct {
    const pid_gains_t *gains;                      /* zajednicki koeficijenti */

    volatile float reference;                      /* zadata brzina tocka [rad/s] */
    volatile float feedback;                       /* izmerena brzina tocka [rad/s] */
    volatile float error;                          /* reference - feedback [rad/s] */
    volatile float error_previous;                 /* greska iz prethodnog tika */

    volatile float integrator;                     /* akumulirani I clan [%] */
    volatile float integrator_limit;               /* +- granica I clana [%] */

    volatile float derivative_raw;                 /* error - error_previous */
    volatile float derivative_filtered;            /* posle niskopropusnog filtera */

    volatile float command;                        /* izlaz regulatora [% maks. PWM-a] */
    volatile float command_limit;                  /* +- granica izlaza [%] */
} wheel_pid_t;

/* Podesi zajednicke koeficijente. derivative_filter_fc_hz i control_freq_hz
 * sluze da se izracuna koeficijent filtera D clana. */
void wheel_pid_gains_init(pid_gains_t *gains, float kp, float ki, float kd,
                          float derivative_filter_fc_hz, float control_freq_hz);

/* Poveze regulator sa zajednickim koeficijentima, postavi granice, obrise stanje. */
void wheel_pid_init(wheel_pid_t *pid, const pid_gains_t *gains,
                    float command_limit_pct, float integrator_limit_pct);

/* Obrise stanje (greska, I clan, D filter, izlaz) - koeficijenti ostaju. */
void wheel_pid_reset(wheel_pid_t *pid);

/* Jedan korak regulatora. Pozivati tacno control_freq_hz puta u sekundi.
 * Vraca command u % (-100..+100). */
float wheel_pid_update(wheel_pid_t *pid, float reference, float feedback);

#endif /* LIB_PID_H_ */
