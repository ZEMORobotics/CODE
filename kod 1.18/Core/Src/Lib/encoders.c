#include "encoders.h"
#include "main.h"

/* ---- dijagnostika, gleda se u Live Expressions ---- */
volatile uint32_t dbg_rcc_tim2 = 0;    /* 1 = TIM2 ima takt */
volatile uint32_t dbg_rcc_tim3 = 0;
volatile uint32_t dbg_smcr2_pre = 0;   /* SMCR pre naseg upisa */
volatile uint32_t dbg_smcr2_post = 0;  /* SMCR posle naseg upisa */
volatile uint32_t dbg_smcr3_post = 0;

/* Pinovi: alternativna funkcija + pull-up (CubeMX ostavlja NOPULL). */
static void encoder_pins_pullup(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pull  = GPIO_PULLUP;

    g.Alternate = GPIO_AF1_TIM2;
    g.Pin = ENC1_A_Pin;  HAL_GPIO_Init(ENC1_A_GPIO_Port, &g);
    g.Pin = ENC1_B_Pin;  HAL_GPIO_Init(ENC1_B_GPIO_Port, &g);

    g.Alternate = GPIO_AF2_TIM3;
    g.Pin = ENC2_A_Pin;  HAL_GPIO_Init(ENC2_A_GPIO_Port, &g);
    g.Pin = ENC2_B_Pin;  HAL_GPIO_Init(ENC2_B_GPIO_Port, &g);
}

/* Enkoderski rezim upisan direktno u registre, bez HAL-a.
 * SMS = 011 (TI12), CC1S = CC2S = 01 (ulazi na TI1/TI2), CC1E/CC2E upaljeni. */
static void encoder_mode_raw(TIM_TypeDef *T)
{
    T->CR1   &= ~TIM_CR1_CEN;                       /* stani dok menjas  */
    T->SMCR   = (T->SMCR & ~(uint32_t)0x7U) | 0x3U; /* SMS = 3           */
    T->CCMR1  = (T->CCMR1 & ~(uint32_t)0x0303U) | 0x0101U;  /* CC1S, CC2S = 01 */
    T->CCER  &= ~(uint32_t)(TIM_CCER_CC1P | TIM_CCER_CC1NP |
                            TIM_CCER_CC2P | TIM_CCER_CC2NP);
    T->CCER  |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
    T->CNT    = 0;
    T->CR1   |= TIM_CR1_CEN;                        /* kreni             */
}

void encoder_init(void)
{
    encoder_pins_pullup();

    dbg_rcc_tim2  = (RCC->APB1LENR & RCC_APB1LENR_TIM2EN) ? 1U : 0U;
    dbg_rcc_tim3  = (RCC->APB1LENR & RCC_APB1LENR_TIM3EN) ? 1U : 0U;
    dbg_smcr2_pre = ENC_LEFT_TIMER.Instance->SMCR & 0x7U;

    encoder_mode_raw(ENC_LEFT_TIMER.Instance);
    encoder_mode_raw(ENC_RIGHT_TIMER.Instance);

    dbg_smcr2_post = ENC_LEFT_TIMER.Instance->SMCR  & 0x7U;
    dbg_smcr3_post = ENC_RIGHT_TIMER.Instance->SMCR & 0x7U;

    encoder_reset();
}

void encoder_reset(void)
{
    ENC_LEFT_TIMER.Instance->CNT = 0;
    ENC_RIGHT_TIMER.Instance->CNT = 0;
}

int32_t encoder_get_count_left_motor(void)
{
    return (int32_t)ENC_LEFT_TIMER.Instance->CNT;
}

int32_t encoder_get_count_right_motor(void)
{
    return (int32_t)ENC_RIGHT_TIMER.Instance->CNT;
}
