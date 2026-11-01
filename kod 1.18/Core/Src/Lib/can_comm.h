/*
 * can_comm.h - CAN protokol Raspberry (ros2 control) <-> STM kretacka
 *
 * ID-jevi su preuzeti iz ros2 enum-a CanID i moraju biti identicni sa obe strane.
 * PAZNJA: vrednosti su DEKADNE (100..152), ne heksadekadne. Ranije je ovde
 * pisalo 0x100..0x152 sto je 256..338 - ploca i Raspberry se tada ne bi culi.
 *
 *  Raspberry -> STM
 *    100  WHEEL_COMMAND_VEL   float speed_left, float speed_right  (rad/s)   8 B
 *    103  PID_KP    104 PID_KI    105 PID_KD                                 4 B
 *         koeficijenti regulatora brzine tocka; jedan skup za oba tocka
 *    106  PID_KP_ROT  107 PID_KI_ROT  108 PID_KD_ROT                         4 B
 *         koeficijenti regulatora za krivolinijsko kretanje (stara
 *         linear/angular sema). Sa regulatorima po tocku nema gde da se
 *         primene - primaju se i cuvaju da ros2 moze da ih salje bez greske.
 *    109  MOTORS_ON       110 MOTORS_OFF                          1 B (payload se ignorise)
 *    111  FEEDBACK_ON     112 FEEDBACK_OFF                        1 B
 *    113  REFERENCE_ON    114 REFERENCE_OFF                       1 B
 *    115  RESET_ENCODERS  116 RESET_PID                           1 B
 *
 *  STM -> Raspberry
 *    101  WHEEL_FEEDBACK_VEL  float speed_left, float speed_right  (rad/s, izmereno
 *                             i filtrirano)                        8 B  100 Hz, samo kad je feedback ukljucen
 *    102  WHEEL_POSITION      int32 encoder_left, int32 encoder_right (ukupni tickovi)
 *                                                                  8 B  100 Hz, isti uslov
 *    150  FRONT_IR   uint8 (0/1)                                   1 B  samo pri promeni, 10x
 *    151  BACK_IR    uint8 (0/1)                                   1 B  samo pri promeni, 10x
 *    152  CINCH      uint8 (0=utaknut, 1=izvucen/START)            1 B  pri promeni 10x + na 1 s
 *
 *  152 je broadcast - cuju ga i Raspberry i sve ostale ploce na busu.
 *  Ponavlja se svake sekunde da bi i ploca koja se kasnije upali saznala
 *  da je mec vec poceo.
 *
 *  Sve brzine na CAN-u su rad/s i regulator radi u rad/s - nema konverzije.
 *  Float-ovi sa CAN-a se proveravaju (NaN / beskonacno se odbacuje), a zadata
 *  brzina se odseca na +-MAX_WHEEL_SPEED_RADS.
 *
 *  Bajtovi u float/int32 su BIG-ENDIAN (kao u referentnom PIC32 protokolu).
 *  Promeni CAN_LITTLE_ENDIAN na 1 ako Raspberry strana pakuje little-endian.
 */
#ifndef LIB_CAN_COMM_H_
#define LIB_CAN_COMM_H_

#include <stdint.h>
#include <stdbool.h>

#define CAN_LITTLE_ENDIAN        0

/* Identicno ros2 strani (enum CanID). Dekadne vrednosti. */
enum CanID {
    WHEEL_COMMAND_VEL  = 100,
    WHEEL_FEEDBACK_VEL = 101,
    WHEEL_POSITION     = 102,
    PID_KP             = 103,
    PID_KI             = 104,
    PID_KD             = 105,
    PID_KP_ROT         = 106,
    PID_KI_ROT         = 107,
    PID_KD_ROT         = 108,
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

/* Opseg koji propusta hardverski filter FDCAN-a (sve ostalo se odbacuje
 * pre nego sto stigne do procesora). */
#define CAN_ID_FILTER_MIN        WHEEL_COMMAND_VEL   /* 100 */
#define CAN_ID_FILTER_MAX        CINCH               /* 152 */

#define CAN_IR_BURST_COUNT       10     /* koliko puta poslati IR poruku pri promeni */
#define CAN_CINCH_BURST_COUNT    10     /* isto za cinc */
#define CAN_CINCH_HEARTBEAT_MS   1000   /* ponavljanje stanja cinca; 0 = bez ponavljanja */
#define CAN_CMD_TIMEOUT_MS       500    /* 0 = iskljuceno. Bez poruke 100 ovoliko ms -> zadata brzina 0 */

/* Stanje koje puni CAN prekid, cita kontrolna petlja (TIM7). Brzine su rad/s. */
extern volatile float can_ref_speed_left;
extern volatile float can_ref_speed_right;
extern volatile bool  can_motors_on;
extern volatile bool  can_feedback_on;
extern volatile bool  can_reference_on;

/* Koeficijenti regulatora za krivolinijsko kretanje (106..108). Primaju se i
 * cuvaju, ali sa regulatorima po tocku nemaju gde da se primene. */
extern volatile float can_kp_rot, can_ki_rot, can_kd_rot;

/* Stanje cinca: 0 = utaknut, 1 = izvucen (mec poceo). Moze se citati i lokalno. */
extern volatile uint8_t can_cinch_state;

void can_comm_init(void);        /* filteri, start, prekidi */
void can_comm_startup_flush(void);/* isprazni RX, posalji nule; pozvati pre glavne petlje */

/* Pozivati stalno iz glavne petlje: salje feedback, IR burst, proverava timeout */
void can_comm_task(void);

/* Iz TIM7 prekida, FEEDBACK_FREQ_HZ puta u sekundi - samo oznaci da je vreme za slanje */
void can_comm_feedback_due(void);

#endif /* LIB_CAN_COMM_H_ */
