/*
 * protocol.h
 *
 *  Created on: Nov 5, 2026
 *      Author: Vladimir
 */

#ifndef SRC_LIB_PROTOCOL_H_
#define SRC_LIB_PROTOCOL_H_

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define CAN_LITTLE_ENDIAN 0 // 1 - Little Endian, 0 - Big Endian

enum CanID {
    WHEEL_COMMAND_VEL  = 100,
    WHEEL_FEEDBACK_VEL = 101,
    WHEEL_POSITION     = 102,
    PID_KP             = 103,
    PID_KI             = 104,
    PID_KD             = 105,
    MOTORS_ON          = 109,
    MOTORS_OFF         = 110,
    FEEDBACK_ON        = 111,
    FEEDBACK_OFF       = 112,
    REFERENCE_ON       = 113,
    REFERENCE_OFF      = 114,
    RESET_ENCODERS     = 115,
    RESET_PID          = 116,
    FRONT_IR           = 150,
    BACK_IR            = 151,
    CINCH              = 152
};


static inline void pack_u32(uint8_t *b, uint32_t v)
{
#if CAN_LITTLE_ENDIAN
    b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
#else
    b[3] = v & 0xFF; b[2] = (v >> 8) & 0xFF; b[1] = (v >> 16) & 0xFF; b[0] = (v >> 24) & 0xFF;
#endif
}

static inline uint32_t unpack_u32(const uint8_t *b)
{
#if CAN_LITTLE_ENDIAN
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
#else
    return (uint32_t)b[3] | ((uint32_t)b[2] << 8) | ((uint32_t)b[1] << 16) | ((uint32_t)b[0] << 24);
#endif
}

static inline void pack_float(uint8_t *b, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    pack_u32(b, u);
}

static inline float unpack_float(const uint8_t *b)
{
    uint32_t u = unpack_u32(b);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static inline void pack_i32(uint8_t *b, int32_t v)
{
    pack_u32(b, (uint32_t)v);
}


static inline bool unpack_float_checked(const uint8_t *b, float *out)
{
    float f = unpack_float(b);
    if (!isfinite(f))
        return false;
    *out = f;
    return true;
}


#endif /* SRC_LIB_PROTOCOL_H_ */
