/*
 * lowpass.h - najosnovniji niskopropusni filter prvog reda.
 *
 *   filtrirano = a * filtrirano_staro + (1 - a) * sirovo
 *
 * Koeficijent a zavisi od granicne frekvencije fc i periode odabiranja T:
 *
 *   a = e^(-2 * pi * fc * T)          (0 < a < 1)
 *
 * Minus u eksponentu je obavezan: bez njega je a > 1 i filter se raspadne.
 * Manje a = brzi filter (manje kasni, manje filtrira). a = 0 iskljucuje filter.
 *
 *   fc = 20 Hz, T = 2 ms  ->  a = 0.778
 *   fc = 30 Hz, T = 2 ms  ->  a = 0.686
 *
 * Koristi se za D clan PID-a (PID.c) i za izmerenu brzinu tocka (odometry.c).
 */
#ifndef LIB_LOWPASS_H_
#define LIB_LOWPASS_H_
#include <math.h>

static inline float lowpass_coefficient(float fc_hz, float sample_freq_hz)
{
    if (fc_hz <= 0.0f || sample_freq_hz <= 0.0f)
        return 0.0f;                                   /* bez filtriranja */
    return expf(-6.283185307f * fc_hz / sample_freq_hz);
}

static inline float lowpass_step(float coefficient, float filtered_old, float raw)
{
    return coefficient * filtered_old + (1.0f - coefficient) * raw;
}

#endif /* LIB_LOWPASS_H_ */
