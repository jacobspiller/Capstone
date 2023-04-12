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
#define PWM1_PIN		GPIO_Pin_2
#define PWM2_PIN		GPIO_Pin_3
#define PWM3_PIN		GPIO_Pin_4

void SdkDelayMs(volatile uint32_t lTimeMs);
void LED_Init(void);
void GPIO_Configuration(uint16_t x, uint16_t y, uint16_t z);
void set_neutral(void);
void MFT_Configuration(uint16_t joint1, uint16_t joint2, uint16_t joint3);
void joint_set(uint16_t joint1, uint16_t joint2, uint16_t joint3);

#endif /* USER_INC_GPIO_H_ */
