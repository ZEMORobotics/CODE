/*
 *   filtered = a * filtrirano_staro + (1 - a) * raw
 *   a = e^(-2 * pi * f_c * T)          (0 < a < 1)
 *
 *   f_c = 20 Hz, T = 2ms  ->  a = 0.778
 *   f_c = 30 Hz, T = 2ms  ->  a = 0.686
 */
#ifndef LIB_LOWPASS_H_
#define LIB_LOWPASS_H_

#include <math.h>
#define TWO_PI_F 6.283185307179586f

static inline float lowpass_coefficient(float fc_hz, float sample_freq_hz)
{
    if (fc_hz <= 0.0f || sample_freq_hz <= 0.0f) return 0.0f;

    return expf(-(TWO_PI_F  * (fc_hz / sample_freq_hz)));
}

static inline float lowpass_step(float coefficient, float filtered_old, float raw)
{
    return coefficient * filtered_old + (1.0f - coefficient) * raw;
}

#endif /* LIB_LOWPASS_H_ */
