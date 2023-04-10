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
#include "BlueNRG1_mft.h"
#include "bluenrg_x_device.h"
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"


//extern void SdkDelayMs(volatile uint32_t lTimeMs);

void LED_Init(void){
	GPIO_InitType GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = POWER_LED | BLUETOOTH_LED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = ENABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_WriteBit(POWER_LED, LED_ON);
	GPIO_WriteBit(BLUETOOTH_LED, LED_OFF);
}

/**
  * @brief  GPIO Configuration.
  *          Configure outputs GPIO pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitType GPIO_InitStructure;

  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

  /* Configure PWM pins */
  GPIO_InitStructure.GPIO_Pin = PWM1_PIN | PWM2_PIN | PWM3_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init( &GPIO_InitStructure);

}

/**
  * @brief  MFT_Configuration.
  * @param  None
  * @retval None
  */
void MFT_Configuration(uint16_t joint1, uint16_t joint2, uint16_t joint3)
{
  NVIC_InitType NVIC_InitStructure;
  MFT_InitType timer_init;

  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1, ENABLE);

  MFT_StructInit(&timer_init);

  timer_init.MFT_Mode = MFT_MODE_1;

#if (HS_SPEED_XTAL == HS_SPEED_XTAL_32MHZ)
	timer_init.MFT_Prescaler = 160-1;      /* 5 us clock */
#elif (HS_SPEED_XTAL == HS_SPEED_XTAL_16MHZ)
	timer_init.MFT_Prescaler = 80-1;       /* 5 us clock */
#endif

  /* MFT1 configuration */
  timer_init.MFT_Clock1 = MFT_PRESCALED_CLK;
  timer_init.MFT_Clock2 = MFT_NO_CLK;
  timer_init.MFT_CRA = joint1*(400/360) - 1;  	   /* 1.5 ms high duration */
  timer_init.MFT_CRB = 4000 - 1;       /* 20 ms low duration (50hz)*/
  MFT_Init(MFT1, &timer_init);

  /* Enable MFT1 Interrupt 1 */
  NVIC_InitStructure.NVIC_IRQChannel = MFT1A_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void joint_set(uint16_t joint1_angle, uint16_t joint2_angle, uint16_t joint3_angle){
		/* GPIO configuration */
	   GPIO_Configuration();

	   /* MFT configuration */
	   MFT_Configuration(joint1_angle, joint2_angle, joint3_angle);

	   /* Connect PWM output from MFT1 to TnA pin (PWM0) */
	   MFT_TnXEN(MFT1, MFT_TnB, ENABLE);

	   /** Enable the MFT interrupt */
	   MFT_EnableIT(MFT1, MFT_IT_TNB, ENABLE);

	   /* Start MFT timers */
	   MFT_Cmd(MFT1, ENABLE);


	   MFT_EnableIT(MFT1, MFT_IT_TNB, DISABLE);
	   MFT_Cmd(MFT1, DISABLE);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif
