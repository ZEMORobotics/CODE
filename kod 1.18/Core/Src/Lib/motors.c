#include "motors.h"

static float max_duty_pct = MOTOR_MAX_VOLTAGE / MOTOR_SUPPLY_VOLTAGE_DEFAULT * 100.0f;
static float max_motor_voltage = MOTOR_MAX_VOLTAGE;

void PWM_Init(void)
{
    Motor_Init();

    float voltage = ADC_ReadVbat();

    if(isfinite(voltage) && voltage >= MOTOR_SUPPLY_VOLTAGE_MIN && voltage <= MOTOR_SUPPLY_VOLTAGE_MAX){
		max_duty_pct = (MOTOR_MAX_VOLTAGE / voltage) * 100.0f;

		if (max_duty_pct > MOTOR_ABSOLUTE_MAX_DUTY)
			max_duty_pct = MOTOR_ABSOLUTE_MAX_DUTY;
	}

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

static inline float command_to_duty(float percent)
{
    if (percent < 0.0f)   percent = 0.0f;
    if (percent > 100.0f)    percent = 100.0f;

    return (percent / 100.0f) * max_duty_pct;
}

void PWM_SetCommand_Left(float percent)
{
    if (percent >= 0.0f) {
        Motor_SetDir(0);            /* Left is inverted: 0 = forward*/
    } else {
        Motor_SetDir(1);
        percent = -percent;
    }
    Motor_SetDuty(command_to_duty(percent));
}

void PWM_SetCommand_Right(float percent)
{
    if (percent >= 0.0f) {
        Motor2_SetDir(1);           /* Right: 1 = forward */
    } else {
        Motor2_SetDir(0);
		percent = -percent;
	}
    Motor2_SetDuty(command_to_duty(percent));
}
