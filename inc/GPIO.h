/*
 * GPIO.h
 *
 *  Created on: Jun 27, 2022
 *      Author: Mona
 */

#ifndef USER_INC_GPIO_H_
#define USER_INC_GPIO_H_

#include <bluenrg_x_device.h>
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#define POWER_LED 		GPIO_Pin_0
#define BLUETOOTH_LED 	GPIO_Pin_1

void LED_Configuration(void);
//void SdkDelayMs(volatile uint32_t lTimeMs);
void ToggleLED(void);
void LED_Init(void);

#endif /* USER_INC_GPIO_H_ */
