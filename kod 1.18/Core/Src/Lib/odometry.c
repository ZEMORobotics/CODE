#include "odometry.h"
#include "lowpass.h"
#if ODOMETRY_ON_STM
#include <math.h>
#endif

/* Sve sto pise TIM7 prekid a cita glavna petlja / CAN je volatile. */
static int32_t last_count_left  = 0;     /* poslednje ocitanje hardverskog brojaca */
static int32_t last_count_right = 0;

static volatile int32_t delta_ticks_left  = 0;   /* tickovi u poslednjem tiku petlje */
static volatile int32_t delta_ticks_right = 0;
static volatile int32_t total_ticks_left  = 0;   /* ukupno od reseta, int32 */
static volatile int32_t total_ticks_right = 0;

static volatile float speed_left_raw  = 0.0f;    /* rad/s iz jednog tika */
static volatile float speed_right_raw = 0.0f;
static volatile float speed_left      = 0.0f;    /* rad/s posle filtera */
static volatile float speed_right     = 0.0f;

static float speed_filter_coefficient = 0.0f;    /* a = e^(-2*pi*fc*T) */

#if ODOMETRY_ON_STM
static float wheel_radius_mm = WHEEL_RADIUS_MM;
static float wheel_base_mm   = WHEEL_BASE_MM;
static float x = 0.0f, y = 0.0f, fi = 0.0f, unwrapped_fi = 0.0f;
#endif

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

#if ODOMETRY_ON_STM
    set_x_y(0.0f, 0.0f, 0.0f);
#endif
}

void odometry_update(void)
{
    int32_t count_left  = encoder_get_count_left_motor();
    int32_t count_right = encoder_get_count_right_motor();

    /* Razlika se racuna u 16 bita pa prosiruje sa znakom. To radi i za
     * 32-bitni TIM2 i za 16-bitni TIM3, i prelazak brojaca preko nule ne
     * pravi skok (dokle god je manje od 32768 tickova po tiku, a to je
     * nemoguce - to bi bilo 4 obrtaja za 2 ms). */
    int32_t dl = (int16_t)((uint16_t)count_left  - (uint16_t)last_count_left)  * ENC_LEFT_SIGN;
    int32_t dr = (int16_t)((uint16_t)count_right - (uint16_t)last_count_right) * ENC_RIGHT_SIGN;

    last_count_left  = count_left;
    last_count_right = count_right;

    delta_ticks_left  = dl;
    delta_ticks_right = dr;
    total_ticks_left  += dl;
    total_ticks_right += dr;

    /* tickovi po tiku -> rad/s:  d * (2*pi / ticks_per_rev) * f_petlje */
    speed_left_raw  = (float)dl * RAD_PER_TICK * (float)CONTROL_FREQ_HZ;
    speed_right_raw = (float)dr * RAD_PER_TICK * (float)CONTROL_FREQ_HZ;

    speed_left  = lowpass_step(speed_filter_coefficient, speed_left,  speed_left_raw);
    speed_right = lowpass_step(speed_filter_coefficient, speed_right, speed_right_raw);

#if ODOMETRY_ON_STM
    /* ---- pozicija robota - iskljuceno, radi je ros2 ---------------------- */
    float ds_l = (float)dl * RAD_PER_TICK * wheel_radius_mm;   /* mm */
    float ds_r = (float)dr * RAD_PER_TICK * wheel_radius_mm;
    float ds   = (ds_r + ds_l) / 2.0f;
    float dfi  = (ds_l - ds_r) / wheel_base_mm;
    float fi_mid = fi + (dfi / 2.0f);

    x += ds * cosf(fi_mid);
    y += ds * sinf(fi_mid);
    fi += dfi;
    unwrapped_fi += dfi;
    while (fi >  M_PI) fi -= 2.0f * M_PI;
    while (fi < -M_PI) fi += 2.0f * M_PI;
#endif
}

float get_wheel_speed_left(void)      { return speed_left; }
float get_wheel_speed_right(void)     { return speed_right; }
float get_wheel_speed_left_raw(void)  { return speed_left_raw; }
float get_wheel_speed_right_raw(void) { return speed_right_raw; }

int32_t get_encoder_total_left(void)  { return total_ticks_left; }
int32_t get_encoder_total_right(void) { return total_ticks_right; }

#if ODOMETRY_ON_STM
float get_x(void)  { return x; }
float get_y(void)  { return y; }
float get_fi(void) { return fi; }
float get_unwrapped_fi(void) { return unwrapped_fi; }

void set_x_y(float x_p, float y_p, float fi_p)
{
    x = -y_p;
    y = -x_p;
    fi = fi_p;
    unwrapped_fi = fi_p;
}
#endif
