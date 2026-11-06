#include "can_comm.h"

volatile float can_ref_speed_left  = 0.0f;
volatile float can_ref_speed_right = 0.0f;

volatile bool can_motors_on    = true;
volatile bool can_feedback_on  = false;
volatile bool can_reference_on = false;

volatile uint8_t can_cinch_state = 0;

static volatile bool feedback_due   = false;
static volatile uint32_t last_cmd_tick  = 0;
static volatile bool cmd_timed_out  = false;


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

static inline bool can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0)
    {
        return false;
    }

    FDCAN_TxHeaderTypeDef tx;
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = len * FDCAN_DLC_BYTES_1;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;

    bool ok = (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)data) == HAL_OK);
    return ok;
}

static inline bool send_speed_feedback(void)
{
    uint8_t buf[8];
    pack_float(buf,     get_wheel_speed_left());
    pack_float(buf + 4, get_wheel_speed_right());
    return can_send(WHEEL_FEEDBACK_VEL, buf, 8);
}

static inline bool send_encoder_feedback(void)
{
    uint8_t buf[8];
    pack_i32(buf,     get_encoder_total_left());
    pack_i32(buf + 4, get_encoder_total_right());
    return can_send(WHEEL_POSITION, buf, 8);
}

static void process_msg(uint32_t id, const uint8_t *d)
{
	float g;

    switch (id)
    {
    case WHEEL_COMMAND_VEL:
        last_cmd_tick = HAL_GetTick();
        cmd_timed_out = false;
        if (!can_reference_on)
        	return;

        float wl, wr;
        if (!unpack_float_checked(d, &wl) || !unpack_float_checked(d + 4, &wr))
            return;

        can_ref_speed_left  = wl;
        can_ref_speed_right = wr;
        break;


    case PID_KP:
        if (unpack_float_checked(d, &g))
        	wheel_gains.proportional_gain = g;
        break;

    case PID_KI:
        if (unpack_float_checked(d, &g))
        	wheel_gains.integral_gain = g;
        break;

    case PID_KD:
        if (unpack_float_checked(d, &g))
        	wheel_gains.derivative_gain = g;
        break;

    case MOTORS_ON:
        wheel_pid_reset(&pid_left);
        wheel_pid_reset(&pid_right);
        can_ref_speed_left = 0.0f;
        can_ref_speed_right = 0.0f;
        PWM_Enable(true);
        can_motors_on = true;
        break;

    case MOTORS_OFF:
        can_motors_on = false;
        can_ref_speed_left = 0.0f;
        can_ref_speed_right = 0.0f;
        PWM_Enable(false);
        wheel_pid_reset(&pid_left);
        wheel_pid_reset(&pid_right);
        break;

    case FEEDBACK_ON:
    	can_feedback_on = true;
    	break;

    case FEEDBACK_OFF:
    	can_feedback_on = false;
    	break;

    case REFERENCE_ON:
    	can_reference_on = true;
    	break;

    case REFERENCE_OFF:
        can_reference_on = false;
        can_ref_speed_left = 0.0f;
        can_ref_speed_right = 0.0f;
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

        process_msg(rx.Identifier, data);
    }
}

void can_comm_init(void)
{
    FDCAN_FilterTypeDef f;
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = CAN_ID_FILTER_MIN;   /* 100 */
    f.FilterID2    = CAN_ID_FILTER_MAX;   /* 152 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK)
        Error_Handler();

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        Error_Handler();

    last_cmd_tick = HAL_GetTick();
}

void can_comm_startup_flush(void)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, data);

    last_cmd_tick = HAL_GetTick();

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		Error_Handler();
}


static void din_poll(din_t *d, uint32_t now)
{
    uint8_t raw = (HAL_GPIO_ReadPin(d->port, d->pin) == GPIO_PIN_SET) ? 1 : 0;
    if (!d->active_high)
        raw ^= 1;

    if (raw != d->raw_last) {
        d->raw_last  = raw;
        d->raw_since = now;
    } else if (d->stable != raw && (now - d->raw_since) >= d->debounce_ms) {
        d->stable = raw;
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

    /* Cinch beacon message */
    if (din_cinch.stable != 0xFF && (now - cinch_hb_tick) >= CAN_CINCH_HEARTBEAT_MS) {
        cinch_hb_tick = now;
        if (din_cinch.burst == 0)
            din_cinch.burst = 1;
    }
}


void can_comm_feedback_due(void)
{
    feedback_due = true;
}

void can_comm_task(void)
{
    uint32_t now = HAL_GetTick();

    if (feedback_due) {
        feedback_due = false;
        if (can_feedback_on) {
            send_speed_feedback();
            send_encoder_feedback();
        }
    }

    inputs_check(now);

    if (!cmd_timed_out && (now - last_cmd_tick) > CAN_CMD_TIMEOUT_MS) {
        cmd_timed_out = true;
        can_ref_speed_left = 0.0f;
        can_ref_speed_right = 0.0f;
    }
}
