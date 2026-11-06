#include "odometry.h"

static int32_t last_count_left  = 0;
static int32_t last_count_right = 0;

static volatile int32_t delta_ticks_left  = 0;
static volatile int32_t delta_ticks_right = 0;
static volatile int32_t total_ticks_left  = 0;
static volatile int32_t total_ticks_right = 0;

static volatile float speed_left_raw  = 0.0f;
static volatile float speed_right_raw = 0.0f;
static volatile float speed_left      = 0.0f;
static volatile float speed_right     = 0.0f;

static float speed_filter_coefficient = 0.0f;


void odometry_init(void)
{
    speed_filter_coefficient = lowpass_coefficient(SPEED_LPF_FC_HZ, (float)CONTROL_FREQ_HZ);
    odometry_reset();
}

void odometry_reset(void)
{
    encoder_reset();
    last_count_left  = encoder_get_count_left_motor();
    last_count_right = encoder_get_count_right_motor();

    delta_ticks_left  = 0;   delta_ticks_right = 0;
    total_ticks_left  = 0;   total_ticks_right = 0;
    speed_left_raw    = 0.0f; speed_right_raw  = 0.0f;
    speed_left        = 0.0f; speed_right      = 0.0f;

}

void odometry_update(void)
{
    int32_t count_left  = encoder_get_count_left_motor();
    int32_t count_right = encoder_get_count_right_motor();

    /* Overflow handle*/
    int32_t dl = (int16_t)((uint16_t)count_left  - (uint16_t)last_count_left)  * ENC_LEFT_SIGN;
    int32_t dr = (int16_t)((uint16_t)count_right - (uint16_t)last_count_right) * ENC_RIGHT_SIGN;

    last_count_left  = count_left;
    last_count_right = count_right;

    delta_ticks_left  = dl;
    delta_ticks_right = dr;
    total_ticks_left  += dl;
    total_ticks_right += dr;

    /* rad / s:  d * (2*pi / ticks_per_rev) * f_petlje */
    speed_left_raw  = (float)dl * RAD_PER_TICK * (float)CONTROL_FREQ_HZ;
    speed_right_raw = (float)dr * RAD_PER_TICK * (float)CONTROL_FREQ_HZ;

    speed_left  = lowpass_step(speed_filter_coefficient, speed_left,  speed_left_raw);
    speed_right = lowpass_step(speed_filter_coefficient, speed_right, speed_right_raw);
}

float get_wheel_speed_left(void)      { return speed_left; }
float get_wheel_speed_right(void)     { return speed_right; }
float get_wheel_speed_left_raw(void)  { return speed_left_raw; }
float get_wheel_speed_right_raw(void) { return speed_right_raw; }

int32_t get_encoder_total_left(void)  { return total_ticks_left; }
int32_t get_encoder_total_right(void) { return total_ticks_right; }
