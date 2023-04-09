
#ifndef USER_INC_ADC_H_
#define USER_INC_ADC_H_

#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "BlueNRG1_adc.h"
#define ADC_READING_OFFSET		0.020 //ADC reading value is always 0.02 smaller than actual values

void Batt_ADC_Configuration(void);
uint16_t ADC_Raw_Value(uint8_t input);
float ADC_Voltage_Value(uint8_t input);
void InitGPIO_Output(uint32_t GPIOPort);
void InitGPIO_Input(uint32_t GPIOPort);
void InitGPIO_LowPower(uint32_t GPIOPort);

BOOL Check_Batt_Voltage();
ADC_InitType sensorADC;


#endif /* USER_INC_ADC_H_ */
