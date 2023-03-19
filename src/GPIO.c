/*
 * GPIO.c
 *
 *  Created on: Jun 27, 2022
 *      Author: Mona
 */

#include "GPIO.h"
#include "adc.h"
#include "clock.h"
#include "SEGGER_RTT.h"

#include "bluenrg_x_device.h"
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#define LED_PIN			GPIO_Pin_1

extern void SdkDelayMs(volatile uint32_t lTimeMs);

void LED_Configuration(void){
	InitGPIO_Output(LED_PIN);

	/* Turn off LED that is on GPIO 1, reset should set the bit to 0 but this is just to be sure*/
	GPIO_WriteBit(LED_PIN, LED_OFF);

	    /* Configure SysTick to generate interrupt */
	 //   SysTick_Config(SYST_CLOCK/1000 - 1);
}


void ToggleLED(void){
	LED_Configuration();

	GPIO_WriteBit(LED_PIN,LED_ON);
	SdkDelayMs(50); //turns LED on for 2s then off
	GPIO_WriteBit(LED_PIN, LED_OFF);
	SdkDelayMs(50);

}
