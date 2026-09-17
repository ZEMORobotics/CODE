/*
 * motors.c
 *
 * Motor 1 = levi tocak (PE9),  Motor 2 = desni tocak (PE11).
 * Levi motor je fizicki zalemljen obrnuto pa mu je DIR logika invertovana.
 */

#include "motors.h"
#include "motor.h"
#include "motor2.h"

volatile float motor_supply_voltage = MOTOR_SUPPLY_VOLTAGE_DEFAULT;

/* Plafon ispune [%]. Pise ga PWM_Init / PWM_SetSupplyVoltage (glavna petlja),
 * cita TIM7 prekid - zato volatile. Pocetna vrednost = 30 V -> 60 %. */
static volatile float max_duty_pct = MOTOR_MAX_VOLTAGE / MOTOR_SUPPLY_VOLTAGE_DEFAULT * 100.0f;

static bool supply_voltage_valid(float v)
{
    /* "v >= MIN && v <= MAX" je false i za NaN, pa NaN ne prolazi */
    return (v >= MOTOR_SUPPLY_VOLTAGE_MIN) && (v <= MOTOR_SUPPLY_VOLTAGE_MAX);
}

static void recompute_max_duty(void)
{
    float v = motor_supply_voltage;
    if (!supply_voltage_valid(v)) {
        v = MOTOR_SUPPLY_VOLTAGE_DEFAULT;      /* neko je upisao glupost - nazad na sigurno */
        motor_supply_voltage = v;
    }
    float d = MOTOR_MAX_VOLTAGE / v * 100.0f;
    if (d > 100.0f) d = 100.0f;
    max_duty_pct = d;
}

bool PWM_SetSupplyVoltage(float volts)
{
    if (!supply_voltage_valid(volts))
        return false;
    motor_supply_voltage = volts;
    recompute_max_duty();
    return true;
}

float PWM_GetMaxDutyPct(void)
{
    return max_duty_pct;
}

void PWM_Init(void)
{
    recompute_max_duty();
    Motor_Init();
    Motor2_Init();
    Motor_Enable(1);
    Motor2_Enable(1);
}

void PWM_Enable(bool on)
{
    Motor_SetDuty(0.0f);
    Motor2_SetDuty(0.0f);
    Motor_Enable(on ? 1 : 0);
    Motor2_Enable(on ? 1 : 0);
}

void PWM_Stop(void)
{
    Motor_SetDuty(0.0f);
    Motor2_SetDuty(0.0f);
}

/* Poslednja brana pred CCR registrom: |komanda| u % -> ispuna u %, nikad
 * preko max_duty_pct. Hvata i NaN / beskonacno (poredjenje sa NaN je false). */
static float command_to_duty(float percent)
{
    if (!(percent > 0.0f))   percent = 0.0f;
    if (percent > 100.0f)    percent = 100.0f;

    float duty = (percent / 100.0f) * max_duty_pct;
    if (duty > max_duty_pct) duty = max_duty_pct;
    return duty;
}

void PWM_SetCommand_Left(float percent)
{
    if (percent >= 0.0f) {
        Motor_SetDir(0);            /* levi je invertovan: 0 = napred */
    } else {
        Motor_SetDir(1);
        percent = -percent;
    }
    Motor_SetDuty(command_to_duty(percent));
}

void PWM_SetCommand_Right(float percent)
{
    if (percent >= 0.0f) {
        Motor2_SetDir(1);           /* desni: 1 = napred */
    } else {
        Motor2_SetDir(0);
        percent = -percent;
    }
    Motor2_SetDuty(command_to_duty(percent));
}
