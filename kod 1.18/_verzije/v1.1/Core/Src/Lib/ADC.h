/*
 * ADC.h
 *
 *  Created on: Aug 19, 2026
 *      Author: vuleb
 */

#ifndef SRC_LIB_ADC_H_
#define SRC_LIB_ADC_H_

#include "main.h"

/* ================== Podesavanja ================== */
#define VREF        3.300f
#define ADC_MAX     65535.0f


#define R27         100000.0f
#define R28         8760.0f
#define DIVIDER     ((R27 + R28) / R28)

#define N_SAMPLES   16

void ADC_Start(void);
float ADC_ReadVbat(void);

#endif /* SRC_LIB_ADC_H_ */
