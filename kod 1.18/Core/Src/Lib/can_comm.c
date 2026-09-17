#include "can_comm.h"
#include "board_config.h"
#include "main.h"
#include "fdcan.h"
#include "PID.h"
#include "odometry.h"
#include "motors.h"
#include <string.h>
#include <math.h>

extern wheel_pid_t pid_left;     /* main.c */
extern wheel_pid_t pid_right;
extern pid_gains_t wheel_gains;  /* zajednicki koeficijenti oba tocka */

volatile float can_ref_speed_left  = 0.0f;
volatile float can_ref_speed_right = 0.0f;
volatile bool  can_motors_on    = true;
volatile bool  can_feedback_on  = false;   /* po defaultu ne saljemo nazad */
volatile bool  can_reference_on = true;
volatile float can_kp_rot = 0.0f, can_ki_rot = 0.0f, can_kd_rot = 0.0f;
volatile uint8_t can_cinch_state = 0;

static volatile bool     feedback_due   = false;
static volatile uint32_t last_cmd_tick  = 0;
static bool              cmd_timed_out  = false;

/* ------------------------------------------- digitalni ulazi (IR + cinc) */
/*
 * Jedna struktura po ulazu: debounce + burst slanje pri promeni stanja.
 * stable/raw_last = 0xFF znaci "jos nije ocitano" - prvo stabilno ocitavanje
 * se posalje kao pocetno stanje.
 */
typedef struct {
    uint32_t      id;
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       active_high;
    uint16_t      debounce_ms;
    uint8_t       burst_count;
    uint8_t       stable;
    uint8_t       raw_last;
    uint32_t      raw_since;
    uint8_t       burst;
} din_t;

static din_t din_front = {
    FRONT_IR, SENS1_GPIO_Port, SENS1_Pin, IR_ACTIVE_HIGH,
    IR_DEBOUNCE_MS, CAN_IR_BURST_COUNT, 0xFF, 0xFF, 0, 0
};
static din_t din_back = {
    BACK_IR, SENS2_GPIO_Port, SENS2_Pin, IR_ACTIVE_HIGH,
    IR_DEBOUNCE_MS, CAN_IR_BURST_COUNT, 0xFF, 0xFF, 0, 0
};
static din_t din_cinch = {
    CINCH, CINC_GPIO_Port, CINC_Pin, CINC_ACTIVE_HIGH,
    CINC_DEBOUNCE_MS, CAN_CINCH_BURST_COUNT, 0xFF, 0xFF, 0, 0
};

static uint32_t cinch_hb_tick = 0;

/* ---------------------------------------------------------------- pack/unpack */

static void pack_u32(uint8_t *b, uint32_t v)
{
#if CAN_LITTLE_ENDIAN
    b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
#else
    b[3] = v & 0xFF; b[2] = (v >> 8) & 0xFF; b[1] = (v >> 16) & 0xFF; b[0] = (v >> 24) & 0xFF;
#endif
}

static uint32_t unpack_u32(const uint8_t *b)
{
#if CAN_LITTLE_ENDIAN
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
#else
    return (uint32_t)b[3] | ((uint32_t)b[2] << 8) | ((uint32_t)b[1] << 16) | ((uint32_t)b[0] << 24);
#endif
}

static void pack_float(uint8_t *b, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    pack_u32(b, u);
}

static float unpack_float(const uint8_t *b)
{
    uint32_t u = unpack_u32(b);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static void pack_i32(uint8_t *b, int32_t v)
{
    pack_u32(b, (uint32_t)v);
}

/* Float sa CAN-a sme da udje u regulator samo ako je konacan broj.
 * NaN ili beskonacno bi zaglavili integrator i prosli kroz svako klestenje. */
static bool unpack_float_checked(const uint8_t *b, float *out)
{
    float f = unpack_float(b);
    if (!isfinite(f))
        return false;
    *out = f;
    return true;
}

static float clamp_speed(float w)
{
    if (w >  MAX_WHEEL_SPEED_RADS) return  MAX_WHEEL_SPEED_RADS;
    if (w < -MAX_WHEEL_SPEED_RADS) return -MAX_WHEEL_SPEED_RADS;
    return w;
}

/* HAL na H7 kodira DLC kao FDCAN_DLC_BYTES_x (vrednost << 16) u starijim
 * verzijama, a kao cist broj bajtova u novijim - pokrij oba. */
static uint8_t dlc_to_len(uint32_t dlc)
{
    uint32_t v = (dlc > 15) ? (dlc >> 16) : dlc;
    return (v <= 8) ? (uint8_t)v : 8;
}

static uint32_t len_to_dlc(uint8_t len)
{
    switch (len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

/* -------------------------------------------------------------------- TX */

/* Vraca true ako je poruka ubacena u TX FIFO, false ako je FIFO pun (ne blokira). */
static bool can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
        return false;

    FDCAN_TxHeaderTypeDef tx;
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = len_to_dlc(len);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)data) == HAL_OK;
}

static bool send_speed_feedback(void)
{
    uint8_t buf[8];
    /* ista filtrirana brzina [rad/s] koju koristi i PID */
    pack_float(buf,     get_wheel_speed_left());
    pack_float(buf + 4, get_wheel_speed_right());
    return can_send(WHEEL_FEEDBACK_VEL, buf, 8);
}

static bool send_encoder_feedback(void)
{
    uint8_t buf[8];
    pack_i32(buf,     get_encoder_total_left());
    pack_i32(buf + 4, get_encoder_total_right());
    return can_send(WHEEL_POSITION, buf, 8);
}

/* -------------------------------------------------------------------- RX */

static void process_msg(uint32_t id, uint8_t len, const uint8_t *d)
{
    switch (id)
    {
    case WHEEL_COMMAND_VEL:
    {
        if (len < 8) return;
        last_cmd_tick = HAL_GetTick();
        cmd_timed_out = false;
        if (!can_reference_on) return;
        /* ros2 salje rad/s, regulator radi u rad/s - samo provera i odsecanje */
        float wl, wr;
        if (!unpack_float_checked(d, &wl) || !unpack_float_checked(d + 4, &wr))
            return;
        can_ref_speed_left  = clamp_speed(wl);
        can_ref_speed_right = clamp_speed(wr);
        break;
    }

    /* Koeficijenti regulatora po tocku - jedan skup za oba tocka. */
    case PID_KP:
    {
        float g;
        if (len >= 4 && unpack_float_checked(d, &g)) wheel_gains.proportional_gain = g;
        break;
    }
    case PID_KI:
    {
        float g;
        if (len >= 4 && unpack_float_checked(d, &g)) wheel_gains.integral_gain = g;
        break;
    }
    case PID_KD:
    {
        float g;
        if (len >= 4 && unpack_float_checked(d, &g)) wheel_gains.derivative_gain = g;
        break;
    }

    /* Ugaoni regulator (krivolinijsko kretanje) iz stare linear/angular seme.
     * Sa regulatorima po tocku nema gde da se primeni - prima se i cuva da
     * ros2 moze da salje bez greske i da se vidi u debageru. */
    case PID_KP_ROT: if (len >= 4) can_kp_rot = unpack_float(d); break;
    case PID_KI_ROT: if (len >= 4) can_ki_rot = unpack_float(d); break;
    case PID_KD_ROT: if (len >= 4) can_kd_rot = unpack_float(d); break;

    case MOTORS_ON:
        wheel_pid_reset(&pid_left);
        wheel_pid_reset(&pid_right);
        can_ref_speed_left = can_ref_speed_right = 0.0f;
        PWM_Enable(true);
        can_motors_on = true;
        break;

    case MOTORS_OFF:
        can_motors_on = false;
        can_ref_speed_left = can_ref_speed_right = 0.0f;
        PWM_Enable(false);
        wheel_pid_reset(&pid_left);
        wheel_pid_reset(&pid_right);
        break;

    case FEEDBACK_ON:   can_feedback_on = true;   break;
    case FEEDBACK_OFF:  can_feedback_on = false;  break;
    case REFERENCE_ON:  can_reference_on = true;  break;
    case REFERENCE_OFF:
        can_reference_on = false;
        can_ref_speed_left = can_ref_speed_right = 0.0f;
        break;

    case RESET_ENCODERS:
        odometry_reset();
        break;

    case RESET_PID:
        wheel_pid_reset(&pid_left);
        wheel_pid_reset(&pid_right);
        break;

    default:
        break;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK)
            break;
        if (rx.IdType != FDCAN_STANDARD_ID || rx.RxFrameType != FDCAN_DATA_FRAME)
            continue;
        process_msg(rx.Identifier, dlc_to_len(rx.DataLength), data);
    }
}

/* ------------------------------------------------------------------ init */

void can_comm_init(void)
{
    /* Prihvati ceo opseg protokola (100..152 dekadno) u RX FIFO0.
     * Zahteva "Std Filters Nbr" >= 1 u CubeMX-u. */
    FDCAN_FilterTypeDef f;
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = CAN_ID_FILTER_MIN;   /* 100 */
    f.FilterID2    = CAN_ID_FILTER_MAX;   /* 152 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();

    /* Prekidi za RX tek posle startup flush-a, da nule/smece sa starta ne
     * budu obradjeni kao komande. Ukljucuje ih can_comm_startup_flush(). */
    last_cmd_tick = HAL_GetTick();
}

void can_comm_startup_flush(void)
{
    /* 1) isprazni sopstveni RX FIFO neko vreme (kao u referentnom kodu) */
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < CAN_STARTUP_RX_DRAIN_MS)
    {
        while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
            HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, data);
        HAL_Delay(1);
    }

    /* 2) posalji par hiljada nula na feedback ID-jeve da se linux bafer ocisti.
     *    Ukupno trajanje je ograniceno - ako niko na busu ne daje ACK, TX FIFO
     *    ostaje pun i ne sme da nas zadrzi na startu. */
    uint8_t zeros[8] = {0};
    uint32_t t_flush = HAL_GetTick();
    for (uint32_t i = 0; i < CAN_STARTUP_FLUSH_FRAMES; i++)
    {
        uint32_t id = (i & 1) ? WHEEL_POSITION : WHEEL_FEEDBACK_VEL;
        while (!can_send(id, zeros, 8))
        {
            if (HAL_GetTick() - t_flush > CAN_STARTUP_FLUSH_MAX_MS)
                goto flush_done;
        }
    }
flush_done:

    /* 3) sad tek pusti RX prekide */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        Error_Handler();

    last_cmd_tick = HAL_GetTick();
}

/* ------------------------------------------------------------------ task */

void can_comm_feedback_due(void)
{
    feedback_due = true;
}

/*
 * Ocita pin, propusti kroz debounce i - ako se stanje stvarno promenilo -
 * naredi burst slanje. Salje najvise jednu poruku po pozivu da ne zaglavi
 * TX FIFO ostalim porukama.
 */
static void din_poll(din_t *d, uint32_t now)
{
    uint8_t raw = (HAL_GPIO_ReadPin(d->port, d->pin) == GPIO_PIN_SET) ? 1 : 0;
    if (!d->active_high)
        raw ^= 1;

    if (raw != d->raw_last) {
        d->raw_last  = raw;          /* nije se jos smirilo - restartuj brojac */
        d->raw_since = now;
    } else if (d->stable != raw && (now - d->raw_since) >= d->debounce_ms) {
        d->stable = raw;             /* stabilno dovoljno dugo - prihvati */
        d->burst  = d->burst_count;
    }

    if (d->burst && can_send(d->id, (uint8_t *)&d->stable, 1))
        d->burst--;
}

static void inputs_check(uint32_t now)
{
    din_poll(&din_front, now);
    din_poll(&din_back,  now);
    din_poll(&din_cinch,  now);

    can_cinch_state = (din_cinch.stable == 1) ? 1 : 0;

#if CAN_CINCH_HEARTBEAT_MS > 0
    /* Ponovi stanje cinca povremeno, da ga cuje i ploca koja se kasnije upali */
    if (din_cinch.stable != 0xFF && (now - cinch_hb_tick) >= CAN_CINCH_HEARTBEAT_MS) {
        cinch_hb_tick = now;
        if (din_cinch.burst == 0)
            din_cinch.burst = 1;
    }
#endif
}

/* Posle bus-off FDCAN sam sebe zaustavi (INIT bit) - treba ga ponovo startovati. */
static void bus_off_recovery(void)
{
    FDCAN_ProtocolStatusTypeDef ps;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) == HAL_OK && ps.BusOff)
    {
        HAL_FDCAN_Start(&hfdcan1);
    }
}

void can_comm_task(void)
{
    uint32_t now = HAL_GetTick();

    bus_off_recovery();

    if (feedback_due)
    {
        feedback_due = false;
        if (can_feedback_on)
        {
            send_speed_feedback();
            send_encoder_feedback();
        }
    }

    inputs_check(now);

#if CAN_CMD_TIMEOUT_MS > 0
    if (!cmd_timed_out && (now - last_cmd_tick) > CAN_CMD_TIMEOUT_MS)
    {
        cmd_timed_out = true;
        can_ref_speed_left = can_ref_speed_right = 0.0f;
    }
#endif
}
