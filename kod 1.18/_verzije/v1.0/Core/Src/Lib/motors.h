/*
 * motors.h
 *
 * Tanak adapter iznad postojecih Motor_x / Motor2_x funkcija (motor.h, motor2.h).
 *
 * Ulaz je komanda regulatora u procentima maksimalnog dozvoljenog PWM-a
 * (-100..+100). Ovde se ona prevede u smer + ispunu, a plafon ispune se
 * racuna iz napona napajanja:
 *
 *      max_duty [%] = MOTOR_MAX_VOLTAGE / motor_supply_voltage * 100
 *
 * Na 30 V je to 60 % = CCR 1536 od 2560. Komanda +100 % = tacno taj CCR.
 */
#ifndef LIB_MOTORS_H_
#define LIB_MOTORS_H_
#include "board_config.h"
#include <stdbool.h>

/* Napon napajanja motora [V]. Globalna promenljiva, ne makro - kasnije ce se
 * na startu ucitati sa ADC-a preko PWM_SetSupplyVoltage(). Dok se to ne
 * uradi, vazi MOTOR_SUPPLY_VOLTAGE_DEFAULT. */
extern volatile float motor_supply_voltage;

void  PWM_Init(void);
void  PWM_Enable(bool on);            /* nSLEEP oba drajvera + duty 0 */
void  PWM_Stop(void);                 /* duty 0 na oba, drajveri ostaju budni */

void  PWM_SetCommand_Left(float percent);   /* -100..+100 % maks. PWM-a */
void  PWM_SetCommand_Right(float percent);

/* Upisi izmereni napon napajanja i preracunaj plafon ispune.
 * Vraca false (i ne menja nista) ako je vrednost van MOTOR_SUPPLY_VOLTAGE_MIN..MAX. */
bool  PWM_SetSupplyVoltage(float volts);
float PWM_GetMaxDutyPct(void);        /* trenutni plafon ispune [%], za watch */

#endif /* LIB_MOTORS_H_ */
