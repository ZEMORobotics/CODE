#include "ADC.h"

extern ADC_HandleTypeDef hadc1;

float ADC_ReadVbat(void){
	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK){
		return NAN;
	}

	if(HAL_ADC_Start(&hadc1) != HAL_OK)
		return NAN;

	uint32_t sum = 0;
	for(uint32_t i = 0; i < N_SAMPLES; i++){
		if(HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK){
			HAL_ADC_Stop(&hadc1);
			return NAN;
		}

		sum += HAL_ADC_GetValue(&hadc1);
	}

	if (HAL_ADC_Stop(&hadc1) != HAL_OK)
		return NAN;

	float raw = (float)sum / (float)N_SAMPLES;
	float voltage = raw * VREF / ADC_MAX;

	return voltage * DIVIDER;
}
