/*
 *
 *  Raspberry -> STM
 *    100  WHEEL_COMMAND_VEL   float speed_left, float speed_right  (rad/s)   8 B
 *
 *    103  PID_KP 	4B
 *    104 PID_KI 	4B
 *    105 PID_KD 	4B
 *    106  PID_KP_ROT 	4B
 *    107 PID_KI_ROT 	4B
 *    108 PID_KD_ROT 	4B
 *
 *    109  MOTORS_ON       110 MOTORS_OFF                          1 B (payload is ignored)
 *    111  FEEDBACK_ON     112 FEEDBACK_OFF                        1 B
 *    113  REFERENCE_ON    114 REFERENCE_OFF                       1 B
 *    115  RESET_ENCODERS  116 RESET_PID                           1 B
 *
 *  STM -> Raspberry
 *    101  WHEEL_FEEDBACK_VEL  float speed_left, float speed_right  (rad/s) 	8 B
 *    102  WHEEL_POSITION      int32 encoder_left, int32 encoder_right 			8 B
 *
 *    150  FRONT_IR   uint8 (0/1)                                   1 B
 *    151  BACK_IR    uint8 (0/1)                                   1 B
 *    152  CINCH      uint8 (0=cinch_in, 1=cinch_out/START)            1 B
 *
 *  152 is broadcast TODO: CHeck why lol
 *
 *
 *  Bajtovi u float/int32 su BIG-ENDIAN (kao u referentnom PIC32 protokolu). TODO: check this
 */
#ifndef LIB_CAN_COMM_H_
#define LIB_CAN_COMM_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "protocol.h"
#include "board_config.h"
#include "main.h"
#include "fdcan.h"
#include "PID.h"
#include "odometry.h"
#include "motors.h"


#define CAN_ID_FILTER_MIN        WHEEL_COMMAND_VEL   /* 100 */
#define CAN_ID_FILTER_MAX        CINCH               /* 152 */

#define CAN_IR_BURST_COUNT       10     /* IR message send per IR state change */
#define CAN_CINCH_BURST_COUNT    10     /* CInCH message send per state change */
#define CAN_CINCH_HEARTBEAT_MS   1000   /* Cinch Beacon message */
#define CAN_CMD_TIMEOUT_MS       500    /* Watch dog for command velocity */

// TIM7 reads this
extern volatile float can_ref_speed_left;
extern volatile float can_ref_speed_right;
extern volatile bool  can_motors_on;
extern volatile bool  can_feedback_on;
extern volatile bool  can_reference_on;

extern volatile uint8_t can_cinch_state;

void can_comm_init(void);
void can_comm_startup_flush(void);


void can_comm_task(void);

/* TIM7, FEEDBACK_FREQ */
void can_comm_feedback_due(void);

#endif /* LIB_CAN_COMM_H_ */
