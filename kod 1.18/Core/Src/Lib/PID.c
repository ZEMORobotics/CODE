#include "PID.h"

void wheel_pid_gains_init(pid_gains_t *gains, float kp, float ki, float kd, float derivative_filter_fc_hz, float control_freq_hz)
{
    gains->proportional_gain = kp;
    gains->integral_gain     = ki;
    gains->derivative_gain   = kd;
    gains->derivative_filter_coefficient = lowpass_coefficient(derivative_filter_fc_hz, control_freq_hz);
}

void wheel_pid_init(wheel_pid_t *pid, const pid_gains_t *gains, float command_limit_pct, float integrator_limit_pct)
{
    pid->gains            = gains;
    pid->command_limit    = command_limit_pct;
    pid->integrator_limit = integrator_limit_pct;
    wheel_pid_reset(pid);
}

void wheel_pid_reset(wheel_pid_t *pid)
{
    pid->reference           = 0.0f;
    pid->feedback            = 0.0f;
    pid->error               = 0.0f;
    pid->error_previous      = 0.0f;
    pid->integrator          = 0.0f;
    pid->derivative_raw      = 0.0f;
    pid->derivative_filtered = 0.0f;
    pid->command             = 0.0f;
}

float wheel_pid_update(wheel_pid_t *pid, float reference, float feedback)
{
    const pid_gains_t *g = pid->gains;

    pid->reference      = reference;
    pid->feedback       = feedback;
    pid->error_previous = pid->error;
    pid->error          = reference - feedback;

    float proportional_term = g->proportional_gain * pid->error;


    float integrator_new = pid->integrator + g->integral_gain * pid->error;
    if (integrator_new >  pid->integrator_limit) integrator_new =  pid->integrator_limit;
    if (integrator_new < -pid->integrator_limit) integrator_new = -pid->integrator_limit;


    /* D = a * D_filtered + (1 - a) * D_raw */
    pid->derivative_raw      = pid->error - pid->error_previous;
    pid->derivative_filtered = lowpass_step(g->derivative_filter_coefficient, pid->derivative_filtered, pid->derivative_raw);
    float derivative_term = g->derivative_gain * pid->derivative_filtered;

    float command = proportional_term + integrator_new + derivative_term;


    bool integrator_ok = true;
    if(command > pid->command_limit){
    	command = pid->command_limit;
		if (pid->error > 0.0f) integrator_ok = false;

    }else if(command < -pid->command_limit){
		command = -pid->command_limit;
		if (pid->error < 0.0f) integrator_ok = false;
    }

    if (integrator_ok)
        pid->integrator = integrator_new;

    pid->command = command;
    return command;
}
