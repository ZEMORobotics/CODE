/*
 * ADC.c
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */

#include "ADC.h"

extern ADC_HandleTypeDef hadc1;


volatile uint32_t st1 = 111;   // 111 = "nije se pozvalo"
volatile uint32_t st2 = 111;

volatile uint16_t adc_dma_buf[N_SAMPLES]  __attribute__((section(".RAM_D2")));

//void ADC_Start(void)
//{
//    // prvo kalibracija adc-a
//    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
//
//    // pokretanje adc-a preko dma
//    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, N_SAMPLES);
//}

void ADC_Start(void)
{
//     st1 = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
     st2 = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, N_SAMPLES);

    volatile int stop = 0;   // <- ovde stavi breakpoint (klikni levo od broja linije)
    (void)st1; (void)st2;    // da kompajler ne prijavi "unused"
}

float ADC_ReadVbat(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < N_SAMPLES; i++) {  // citanje adc-a uz usrednjavanje vrijednosti
        sum += adc_dma_buf[i];
    }
    float raw   = (float)sum / N_SAMPLES;   // prosecna sirova vrednost
    float v_pin = raw * VREF / ADC_MAX;     // napon na pinu
    return v_pin * DIVIDER;                  // napon baterije (volti)
}
