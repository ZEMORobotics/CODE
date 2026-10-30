/*
 * comm.c
 *
 *  Created on: Aug 21, 2026
 *      Author: vuleb
 */


#include "comm.h"
#include <string.h>

/* Proveri imena u main.c - obicno ovako za tvoj setup */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern I2C_HandleTypeDef  hi2c1;

/* ---- UART4 (PC10 = TX) ---- */
void ComTest_UART4(void)
{
    char msg[] = "UART4 OK\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
}

/* ---- UART7 (PE8 = TX) ---- */
void ComTest_UART7(void)
{
    char msg[] = "UART7 OK\r\n";
    HAL_UART_Transmit(&huart7, (uint8_t*)msg, strlen(msg), 100);
}

/* ---- I2C1 (PB6 = SCL, PB7 = SDA) ---- */
void ComTest_I2C(void)
{
    uint8_t data[] = {0xAA, 0x55};
    uint8_t addr7  = 0x50;   // 7-bit adresa (HAL trazi <<1)
    /* Bez uredjaja na busu bice NACK, ali na analyzeru vidis START + adresu + takt */
    HAL_I2C_Master_Transmit(&hi2c1, addr7 << 1, data, sizeof(data), 100);
}

/* ---- Sve u nizu, sa pauzom da lako uhvatis svaku ---- */
void ComTest_RunAll(void)
{
    ComTest_UART4();
    HAL_Delay(200);

    ComTest_UART7();
    HAL_Delay(200);

    ComTest_I2C();
    HAL_Delay(200);
}
