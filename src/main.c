
/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : MTF/Mode_1/main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : September-2015
* Description        : Code demostrating the MFT Mode 1 functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/



/* Includes ------------------------------------------------------------------*/
#include "bluenrg_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "GPIO.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_conf.h"

#include "GPIO.h"
#include "I2C.h"
#include "IAM20680_use.h"

/** @addtogroup BlueNRG1_StdPeriph_Examples BlueNRG1 Standard Peripheral Examples
  * @{
  */


/** @addtogroup MTFX_Examples MTFX Examples
  * @{
  */

/** @addtogroup MTFX_Mode1 MTFX Mode 1 Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PWM0_PIN			GPIO_Pin_2
#define PWM1_PIN			GPIO_Pin_3

#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

BOOL volatile longTermMode = FALSE;
volatile uint32_t currentAddress = 0;
volatile uint32_t lSystickCounter = 0;

volatile BOOL isIMUAccCalibrated = FALSE;
volatile BOOL isIMUGyroCalibrated = FALSE;

float accReadings[3];
float gyroReadings[3];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
//void MFT_Configuration(void);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program code
  * @param  None
  * @retval None
  */
int main(void)
{
	/*SEGGER Init*/
		SEGGER_RTT_Init();
		SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

	/* System initialization function */
		SystemInit();

	/* Identify BlueNRG1 platform */
		SdkEvalIdentification();
		SdkEvalComUartInit(UART_BAUDRATE);

	/*SysTick Initialization 1ms*/
		SysTick_Config(SYST_CLOCK/1000 - 1);
	/*Initialization*/
		 longTermMode = FALSE;
	/* Configure I2C as master mode */
		I2C_ConfigurationMaster();
	/* GPIO configuration */
		GPIO_Configuration();
	/*LED initialization*/
		LED_Init();
  	/*Initialize IMU*/
		IAM20680_Init();

//  /* MFT configuration */
//		MFT_Configuration();
//
//  /* Connect PWM output from MFT1 to TnA pin (PWM0) */
//  MFT_TnXEN(MFT1, MFT_TnA, ENABLE);
//
//  /* Connect PWM output from MFT2 to TnA pin (PWM1) */
//  MFT_TnXEN(MFT2, MFT_TnA, ENABLE);
//
//
//  /** Enable the MFT interrupt */
//  MFT_EnableIT(MFT1, MFT_IT_TNA | MFT_IT_TNB, ENABLE);
//  MFT_EnableIT(MFT2, MFT_IT_TNA | MFT_IT_TNB, ENABLE);
//
//  /* Start MFT timers */
//  MFT_Cmd(MFT1, ENABLE);
//  MFT_Cmd(MFT2, ENABLE);

  /*IMU*/
    	/* IAM20680 Init and Calibration*/
    	  IAM20680_Init();
    	  if(IAM20680_Calibrate()){
    		  isIMUGyroCalibrated = TRUE;
    	  }
    	  /* Accelerometer Calibration */
    	  accSlope[0] = 1.0023f;
    	  accSlope[1] = 1.00652f;
    	  accSlope[2] = 0.98327f;
    	  accIntercept[0] = -0.00111f;
    	  accIntercept[1] = -0.00644f;
    	  accIntercept[2] = -0.04624f;
    	  isIMUAccCalibrated = TRUE;
    	  SEGGER_RTT_printf(0,"IMU Calibrated 1=TRUE, 0= FALSE. Return:%04x,%04x \n",isIMUAccCalibrated,isIMUGyroCalibrated);

  /* Infinite loop */
  while(1){
	  /*Collect raw data from IMU*/
	  	  if(dataReadyFlag){
	  		  IAM20680_ReadGyroscope(gyroReadings);
	  		  IAM20680_ReadAccelerometer(accReadings);

	  		  /*Raw Data*/
	  		  SEGGER_RTT_printf(0, "%d.%3d    %d.%3d    %d.%3d    %d.%3d    %d.%3d   %d.%3d   \r\n",
	  				  PRINT_INT(accReadings[0]), PRINT_FLOAT(accReadings[0]),
	  				  	  PRINT_INT(accReadings[1]), PRINT_FLOAT(accReadings[1]),
	  						  PRINT_INT(accReadings[2]), PRINT_FLOAT(accReadings[2]),
	  						  	  PRINT_INT(gyroReadings[0]), PRINT_FLOAT(gyroReadings[0]),
	  							  	  PRINT_INT(gyroReadings[1]), PRINT_FLOAT(gyroReadings[1]),
	  								  	  PRINT_INT(gyroReadings[2]), PRINT_FLOAT(gyroReadings[2]));
	  	  }
  }
}


/**
  * @brief  GPIO Configuration.
  *          Configure outputs GPIO pins.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void){
  GPIO_InitType GPIO_InitStructure;

  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

  /* Configure PWM pins */
  GPIO_InitStructure.GPIO_Pin = PWM0_PIN | PWM1_PIN;
  GPIO_InitStructure.GPIO_Mode = Serial1_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init( &GPIO_InitStructure);

}

///**
//  * @brief  MFT_Configuration.
//  * @param  None
//  * @retval None
//  */
//void MFT_Configuration(void)
//{
//  NVIC_InitType NVIC_InitStructure;
//  MFT_InitType timer_init;
//
//  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_MTFX1 | CLOCK_PERIPH_MTFX2, ENABLE);
//
//  MFT_StructInit(&timer_init);
//
//  timer_init.MFT_Mode = MFT_MODE_1;
//
//#if (HS_SPEED_XTAL == HS_SPEED_XTAL_32MHZ)
//	timer_init.MFT_Prescaler = 160-1;      /* 5 us clock */
//#elif (HS_SPEED_XTAL == HS_SPEED_XTAL_16MHZ)
//	timer_init.MFT_Prescaler = 80-1;       /* 5 us clock */
//#endif
//
//  /* MFT1 configuration */
//  timer_init.MFT_Clock1 = MFT_PRESCALED_CLK;
//  timer_init.MFT_Clock2 = MFT_NO_CLK;
//  timer_init.MFT_CRA = 199;//300 - 1;       /* 1.5 ms high duration */
//  timer_init.MFT_CRB = 4000 - 1;       /* 2 ms low duration */
//  MFT_Init(MFT1, &timer_init);
//
//  /* MFT2 configuration */
//  timer_init.MFT_Clock1 = MFT_PRESCALED_CLK;
//  timer_init.MFT_Clock2 = MFT_NO_CLK;
//  timer_init.MFT_CRA = 5000 - 1;        /* 25 ms positive duration */
//  timer_init.MFT_CRB = 1000 - 1;       /* 50 ms negative duration */
//  MFT_Init(MFT2, &timer_init);
//
//  /* Enable MFT2 Interrupt 1 */
//  NVIC_InitStructure.NVIC_IRQChannel = MFT1A_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//  /* Enable MFT2 Interrupt 2 */
//  NVIC_InitStructure.NVIC_IRQChannel = MFT2A_IRQn;
//  NVIC_Init(&NVIC_InitStructure);
//}

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

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/


//
///******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
//* File Name          : BLE_Beacon_main.c
//* Author             : RF Application Team
//* Version            : 1.1.0
//* Date               : 15-January-2016
//* Description        : Code demostrating the BLE Beacon application
//********************************************************************************
//* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
//* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
//* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
//* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
//* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//*******************************************************************************/
//
///**
// * @file BLE_Beacon_main.c
// * @brief This is a BLE beacon demo that shows how to configure a BlueNRG-1,2 device
// * in order to advertise specific manufacturing data and allow another BLE device to
// * know if it is in the range of the BlueNRG-1 beacon device.
// * It also provides a reference example about how using the
// * BLE Over-The-Air (OTA) Service Manager firmware upgrade capability.
// *
//
//* \section WiSE-Studio_project WiSE-Studio project
//  To use the project with WiSE-Studio , please follow the instructions below:
//  -# Open the WiSE-Studio  and select File->Import.
//  -# Select Existing Projects into Workspace.
//  -# Go to Project Explorer section
//  -# Select desired configuration to build from Project->Project->Build Project.
//  -# Select Project->Rebuild All. This will recompile and link the entire application
//  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
//  -# Download the related binary image.
//  -# Alternatively, open the Flasher utility and download the built binary image.
//
//* \section KEIL_project KEIL project
//  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
//  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
//  -# Open the KEIL project
//     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-1\\BLE_Beacon.uvprojx </tt> or
//     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-2\\BLE_Beacon.uvprojx </tt>
//  -# Select desired configuration to build
//  -# Select Project->Rebuild all target files. This will recompile and link the entire application
//  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
//  -# Download the related binary image.
//  -# Alternatively, open the Flasher utility and download the built binary image.
//
//* \section IAR_project IAR project
//  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
//  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
//  -# Open the IAR project
//     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-1\\BLE_Beacon.eww </tt> or
//     <tt> C:\Users\{username}\ST\BlueNRG-1_2 DK x.x.x\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-2\\BLE_Beacon.eww </tt>
//  -# Select desired configuration to build
//  -# Select Project->Rebuild All. This will recompile and link the entire application
//  -# To download the binary image, please connect a SWD HW programmer in your board (if available).
//  -# Select Project->Download and Debug to download the related binary image.
//  -# Alternatively, open the Flasher utility and download the built binary image.
//
//* \subsection Project_configurations Project configurations
//- \c Release - Release configuration
//- \c Use_OTA_ServiceManager - Configuration for Application using OTA Service Manager
//
//
//* \section Board_supported Boards supported
//- \c STEVAL-IDB007V1
//- \c STEVAL-IDB007V2
//- \c STEVAL-IDB008V1
//- \c STEVAL-IDB008V1M
//- \c STEVAL-IDB008V2
//- \c STEVAL-IDB009V1
//
//
// * \section Power_settings Power configuration settings
//@table
//
//==========================================================================================================
//|                                         STEVAL-IDB00XV1                                                |
//----------------------------------------------------------------------------------------------------------
//| Jumper name |            |  Description                                                                |
//| JP1         |   JP2      |                                                                             |
//----------------------------------------------------------------------------------------------------------
//| ON 1-2      | ON 2-3     | USB supply power                                                            |
//| ON 2-3      | ON 1-2     | The supply voltage must be provided through battery pins.                   |
//| ON 1-2      |            | USB supply power to STM32L1, JP2 pin 2 external power to BlueNRG1           |
//
//
//@endtable
//
//* \section Jumper_settings Jumper settings
//@table
//
//========================================================================================================================================================================================
//|                                                                             STEVAL-IDB00XV1                                                                                          |
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//| Jumper name |                                                                Description                                                                                             |
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//| JP1         | 1-2: to provide power from USB (JP2:2-3). 2-3: to provide power from battery holder (JP2:1-2)                                                                          |
//| JP2         | 1-2: to provide power from battery holder (JP1:2-3). 2-3: to provide power from USB (JP1:1-2). Pin2 to VDD  to provide external power supply to BlueNRG-1 (JP1: 1-2)   |
//| JP3         | pin 1 and 2 UART RX and TX of MCU. pin 3 GND.                                                                                                                          |
//| JP4         | Fitted: to provide VBLUE to BlueNRG1. It can be used also for current measurement.                                                                                     |
//| JP5         | Fitted : TEST pin to VBLUE. Not fitted:  TEST pin to GND                                                                                                               |
//
//
//@endtable
//
//* \section Pin_settings Pin settings
//@table
//|            |                                                           Release                                                           ||||||                                                                     Use_OTA_ServiceManager                                                                      ||||||
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |     STEVAL-IDB008V1M     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     GND    |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |           N.A.           |           N.A.           |           N.A.           |         Not Used         |           N.A.           |           N.A.           |
//|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |           N.A.           |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
//|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|   RESETN   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |           N.A.           |           N.A.           |           N.A.           |         Not Used         |           N.A.           |           N.A.           |
//|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
//|    VBLUE   |        N.A.        |        N.A.        |        N.A.        |      Not Used      |        N.A.        |        N.A.        |           N.A.           |           N.A.           |           N.A.           |         Not Used         |           N.A.           |           N.A.           |
//
//@endtable
//
//* \section Serial_IO Serial I/O
//@table
//| Parameter name  | Value            | Unit      |
//----------------------------------------------------
//| Baudrate        | 115200 [default] | bit/sec   |
//| Data bits       | 8                | bit       |
//| Parity          | None             | bit       |
//| Stop bits       | 1                | bit       |
//@endtable
//
//* \section LEDs_description LEDs description
//@table
//|            |                                                           Release                                                           ||||||                                                                                                                           Use_OTA_ServiceManager                                                                                                                            ||||||
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |               STEVAL-IDB007V1              |               STEVAL-IDB007V2              |               STEVAL-IDB008V1              |              STEVAL-IDB008V1M              |               STEVAL-IDB008V2              |               STEVAL-IDB009V1              |
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//|     DL1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
//|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
//|     DL3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |   ON when OTA firmware upgrade is ongoing  |
//|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |                  Not Used                  |
//
//@endtable
//
//
//* \section Buttons_description Buttons description
//@table
//|                |                                                           Release                                                           ||||||                                                                                                                                                                                                               Use_OTA_ServiceManager                                                                                                                                                                                                                ||||||
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |  STEVAL-IDB008V1M  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |                             STEVAL-IDB007V1                            |                             STEVAL-IDB007V2                            |                             STEVAL-IDB008V1                            |                            STEVAL-IDB008V1M                            |                             STEVAL-IDB008V2                            |                             STEVAL-IDB009V1                            |
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |  It allows BLE Beacon application to activate the OTA Service Manager  |
//|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |                                Not Used                                |
//|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |                             Reset BlueNRG1                             |                             Reset BlueNRG1                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |                             Reset BlueNRG2                             |
//
//@endtable
//
//* \section Usage Usage
//
//The Beacon demo configures a BlueNRG-1,2 device in advertising mode (non-connectable mode) with specific manufacturing data.
//It transmits advertisement packets at regular intervals which contain the following manufacturing data:
//@table
//------------------------------------------------------------------------------------------------------------------------
//| Data field              | Description                       | Notes                                                  |
//------------------------------------------------------------------------------------------------------------------------
//| Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
//| ID                      | Beacon ID                         | Fixed value                                            |
//| Length                  | Length of the remaining payload   | NA                                                     |
//| Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
//| Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |
//| Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |
//| Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |
//@endtable
//
// - (1): SIG company identifiers are available on https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
// - NA : Not Applicable;
//NOTEs:
//     - OTA Service Manager support requires to build application by enabling only ST_USE_OTA_SERVICE_MANAGER_APPLICATION=1 (preprocessor, linker) options and through files: OTA_btl.[ch] (refer to Release_with_OTA_ServiceManager IAR workspace).
//
//**/
//
///** @addtogroup BlueNRG1_demonstrations_applications
// *  BlueNRG-1,2 Beacon demo \see BLE_Beacon_main.c for documentation.
// *
// *@{
// */
//
///** @} */
///** \cond DOXYGEN_SHOULD_SKIP_THIS
// */
//
///* Includes ------------------------------------------------------------------*/
//#include <stdio.h>
//#include <string.h>
//#include "BlueNRG1_it.h"
//#include "BlueNRG1_conf.h"
//#include "ble_const.h"
//#include "bluenrg1_stack.h"
//#include "sleep.h"
//#include "SDK_EVAL_Config.h"
//#include "Beacon_config.h"
//#include "OTA_btl.h"
//#include "SEGGER_RTT.h"
//#include "SEGGER_RTT_conf.h"
//
//
///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
//#define BLE_BEACON_VERSION_STRING "1.1.0"
//
///* Set to 1 for enabling Flags AD Type position at the beginning
//   of the advertising packet */
//#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1
//
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
///* Private function prototypes -----------------------------------------------*/
///* Private functions ---------------------------------------------------------*/
//
//void Device_Init(void)
//{
//  SEGGER_RTT_Init();
//  SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
//  uint8_t ret;
//  uint16_t service_handle;
//  uint16_t dev_name_char_handle;
//  uint16_t appearance_char_handle;
//
//  /* Set the TX Power to -2 dBm */
//  ret = aci_hal_set_tx_power_level(1,4);
//  if(ret != 0) {
//    SEGGER_RTT_printf (0,"Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
//    while(1);
//  }
//
//  /* Init the GATT */
//  ret = aci_gatt_init();
//  if (ret != 0)
//    SEGGER_RTT_printf (0,"Error in aci_gatt_init() 0x%04xr\n", ret);
//  else
//    SEGGER_RTT_printf (0,"aci_gatt_init() --> SUCCESS\r\n");
//
//  /* Init the GAP */
//  ret = aci_gap_init(0x01, 0x00, 0x08, &service_handle,
//                     &dev_name_char_handle, &appearance_char_handle);
//  if (ret != 0)
//    SEGGER_RTT_printf (0,"Error in aci_gap_init() 0x%04x\r\n", ret);
//  else
//    SEGGER_RTT_printf (0,"aci_gap_init() --> SUCCESS\r\n");
//}
//
//
///**
//* @brief  Start beaconing
//* @param  None
//* @retval None
//*/
//static void Start_Beaconing(void)
//{
//  uint8_t ret = BLE_STATUS_SUCCESS;
//
//#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
//  /* Set AD Type Flags at beginning on Advertising packet  */
//  uint8_t adv_data[] = {
//      /* Advertising data: Flags AD Type */
//      0x02,
//      0x01,
//      0x06,
//      /* Advertising data: manufacturer specific data */
//      26, //len
//      AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
//      0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
//      0x02,       // ID
//      0x15,       //Length of the remaining payload
//      0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
//      0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
//      0x00, 0x00, // Major number
//      0x00, 0x00, // Minor number
//      0xC8        //2's complement of the Tx power (-56dB)};
//   };
//#else
//   uint8_t manuf_data[] = {
//      26, //len
//      AD_TYPE_MANUFACTURER_SPECIFIC_DATA, //manufacturer type
//      0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
//      0x02,       // ID
//      0x15,       //Length of the remaining payload
//      0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
//      0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
//      0x00, 0x00, // Major number
//      0x00, 0x00, // Minor number
//      0xC8        //2's complement of the Tx power (-56dB)};
//   };
//#endif
//
//  /* disable scan response */
//  ret = hci_le_set_scan_response_data(0,NULL);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    SEGGER_RTT_printf (0,"Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
//    return;
//  }
//  else
//    SEGGER_RTT_printf (0,"hci_le_set_scan_resp_data() --> SUCCESS\r\n");
//
//  /* put device in non connectable mode */
//  ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE,
//                                 0, NULL, 0, NULL, 0, 0);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    SEGGER_RTT_printf (0,"Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
//    return;
//  }
//  else
//    SEGGER_RTT_printf (0,"aci_gap_set_discoverable() --> SUCCESS\r\n");
//
//#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
//  /* Set the  ADV data with the Flags AD Type at beginning of the
//     advertsing packet,  followed by the beacon manufacturer specific data */
//  ret = hci_le_set_advertising_data (sizeof(adv_data), adv_data);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    SEGGER_RTT_printf (0,"Error in hci_le_set_advertising_data() 0x%04x\r\n", ret);
//    return;
//  }
//  else
//    SEGGER_RTT_printf (0,"hci_le_set_advertising_data() --> SUCCESS\r\n");
//#else
//  /* Delete the TX power level information */
//  ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    SEGGER_RTT_printf (0,"Error in aci_gap_delete_ad_type() 0x%04x\r\n", ret);
//    return;
//  }
//  else
//    SEGGER_RTT_printf (0,"aci_gap_delete_ad_type() --> SUCCESS\r\n");
//
//  /* Update the ADV data with the BEACON manufacturing data */
//  ret = aci_gap_update_adv_data(27, manuf_data);
//  if (ret != BLE_STATUS_SUCCESS)
//  {
//    SEGGER_RTT_printf (0,"Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
//    return;
//  }
//  else
//    SEGGER_RTT_printf (0,"aci_gap_update_adv_data() --> SUCCESS\r\n");
//#endif
//}
//
//int main(void)
//{
//  uint8_t ret;
//
//  /* System Init */
//  SystemInit();
//
//  /* Identify BlueNRG-1,2 platform */
//  SdkEvalIdentification();
//
//  /* Init the UART peripheral */
//  SdkEvalComUartInit(UART_BAUDRATE);
//
//  /* BlueNRG-1,2 stack init */
//  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
//  if (ret != BLE_STATUS_SUCCESS) {
//    SEGGER_RTT_printf(0,"Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
//    while(1);
//  }
//
//  /* Init the BlueNRG-1,2 device */
//  Device_Init();
//
//#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
//  /* Initialize the button */
//  SdkEvalPushButtonInit(USER_BUTTON);
//#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
//
//  /* Start Beacon Non Connectable Mode*/
//  Start_Beaconing();
//
//  SEGGER_RTT_printf(0,"BlueNRG-1,2 BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING);
//
//  while(1)
//  {q
//    /* BlueNRG-1,2 stack tick */
//    BTLE_StackTick();
//
//    /* Enable Power Save according the Advertising Interval */
//    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
//
//#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
//    if (SdkEvalPushButtonGetState(USER_BUTTON) == RESET)
//    {
//      OTA_Jump_To_Service_Manager_Application();
//    }
//#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
//  }
//}
//
///* Hardware Error event.
//   This event is used to notify the Host that a hardware failure has occurred in the Controller.
//   Hardware_Code Values:
//   - 0x01: Radio state error
//   - 0x02: Timer overrun error
//   - 0x03: Internal queue overflow error
//   After this event is recommended to force device reset. */
//
//void hci_hardware_error_event(uint8_t Hardware_Code)
//{
//   NVIC_SystemReset();
//}
//
//
///****************** BlueNRG-1,2 Sleep Management Callback ********************************/
//
//SleepModes App_SleepMode_Check(SleepModes sleepMode)
//{
//  if(SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
//    return SLEEPMODE_RUNNING;
//
//  return SLEEPMODE_NOTIMER;
//}
//
///***************************************************************************************/
//
//#ifdef  USE_FULL_ASSERT
//
///**
//* @brief  Reports the name of the source file and the source line number
//*         where the assert_param error has occurred.
//* @param  file: pointer to the source file name
//* @param  line: assert_param error line source number
//*/
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* User can add his own implementation to report the file name and line number,
//  ex: SEGGER_RTT_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
//
//#endif
//
///******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
///** \endcond
// */
//
//
//
/////******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
////* File Name          : SensorDemo_main.c
////* Author             : AMS - RF Application team
////* Version            : V1.2.0
////* Date               : 03-December-2018
////* Description        : BlueNRG-1-2 Sensor Demo main file
////********************************************************************************
////* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
////* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
////* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
////* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
////* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
////* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
////*******************************************************************************/
/////** @addtogroup BlueNRG1_demonstrations_applications
//// * BlueNRG-1,2 SensorDemo \see SensorDemo_main.c for documentation.
//// *
//// *@{
//// */
////
/////** @} */
/////** \cond DOXYGEN_SHOULD_SKIP_THIS
//// */
/////* Includes ------------------------------------------------------------------*/
////#include <stdio.h>
////#include "BlueNRG1_it.h"
////#include "BlueNRG1_conf.h"
////#include "ble_const.h"
////#include "bluenrg1_stack.h"
////#include "SDK_EVAL_Config.h"
////#include "sleep.h"
////#include "sensor.h"
////#include "SensorDemo_config.h"
////#include "OTA_btl.h"
////#include "gatt_db.h"
////#include "SEGGER_RTT.h"
////#include "SEGGER_RTT_conf.h"
////#include "GPIO.h"
////#include "adc.h"
////#include "I2C.h"
////#include "IAM20680.h"
////#include "internalFlash.h"
////#include "sensor.h"
////
/////* Private typedef -----------------------------------------------------------*/
/////* Private define ------------------------------------------------------------*/
////#ifndef DEBUG
////#define DEBUG 0
////#endif
////
////#if DEBUG
////#include <stdio.h>
//////#define SEGGER_RTT_printf(...) SEGGER_RTT_printf(__VA_ARGS__)
////#else
////#define SEGGER_RTT_printf(...)
////#endif
////
////
/////* Private macro -------------------------------------------------------------*/
/////* Private variables ---------------------------------------------------------*/
////
/////* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
////uint8_t Services_Max_Attribute_Records[NUMBER_OF_APPLICATION_SERVICES] = {MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2};
////
/////* Private function prototypes -----------------------------------------------*/
/////* Private functions ---------------------------------------------------------*/
////
////int main(void)
////{
////  uint8_t ret;
////  SEGGER_RTT_Init();
////  SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
////
////  /* System Init */
////  SystemInit();
////
////  /* Identify BlueNRG1 platform */
////  SdkEvalIdentification();
////
////  /* Configure I/O communication channel */
////  SdkEvalComUartInit(UART_BAUDRATE);
////
////  /*Initialize the LEDs*/
////  LED_Init();
////
////  I2C_test();
////
////  /* BlueNRG-1,2 stack init */
////  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
////  if (ret != BLE_STATUS_SUCCESS) {
////	  SEGGER_RTT_SEGGER_RTT_printf(0,"Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
////    while(1);
////  }
////
////  SEGGER_RTT_SEGGER_RTT_printf(0,"Initialization Passed!!\r\n");
////
////#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
////  /* Initialize the button: to be done before Sensor_DeviceInit for avoiding to
////     overwrite pressure/temperature sensor IO configuration when using BUTTON_2 (IO5) */
////  SdkEvalPushButtonInit(USER_BUTTON);
////#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
////
////  /* Sensor Device Init */
////  ret = Sensor_DeviceInit();
////  if (ret != BLE_STATUS_SUCCESS) {
////    SdkEvalLedOn(LED3);
////    while(1);
////  }
////
////
////  while(1)
////  {
////    /* BLE Stack Tick */
////    BTLE_StackTick();
////
////    /* Application Tick */
////    APP_Tick();
////
////    /* Power Save management */
////    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
////
////#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
////    /* Check if the OTA firmware upgrade session has been completed */
////    if (OTA_Tick() == 1)
////    {
////      /* Jump to the new application */
////      OTA_Jump_To_New_Application();
////    }
////#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
////
////#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
////    if (SdkEvalPushButtonGetState(USER_BUTTON) == RESET)
////    {
////      OTA_Jump_To_Service_Manager_Application();
////    }
////#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
////  }/* while (1) */
////}
////
/////* Hardware Error event.
////   This event is used to notify the Host that a hardware failure has occurred in the Controller.
////   Hardware_Code Values:
////   - 0x01: Radio state error
////   - 0x02: Timer overrun error
////   - 0x03: Internal queue overflow error
////   After this event is recommended to force device reset. */
////
////void hci_hardware_error_event(uint8_t Hardware_Code)
////{
////   NVIC_SystemReset();
////}
////
/////**
////  * This event is generated to report firmware error informations.
////  * FW_Error_Type possible values:
////  * Values:
////  - 0x01: L2CAP recombination failure
////  - 0x02: GATT unexpected response
////  - 0x03: GATT unexpected request
////    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect.
////*/
////void aci_hal_fw_error_event(uint8_t FW_Error_Type,
////                            uint8_t Data_Length,
////                            uint8_t Data[])
////{
////  if (FW_Error_Type <= 0x03)
////  {
////    uint16_t connHandle;
////
////    /* Data field is the connection handle where error has occurred */
////    connHandle = LE_TO_HOST_16(Data);
////
////    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER);
////  }
////}
////
/////****************** BlueNRG-1,2 Sleep Management Callback ********************************/
////
////SleepModes App_SleepMode_Check(SleepModes sleepMode)
////{
////  if(request_free_fall_notify || SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
////    return SLEEPMODE_RUNNING;
////
////  return SLEEPMODE_NOTIMER;
////}
////
/////***************************************************************************************/
////
////#ifdef USE_FULL_ASSERT
/////*******************************************************************************
////* Function Name  : assert_failed
////* Description    : Reports the name of the source file and the source line number
////*                  where the assert_param error has occurred.
////* Input          : - file: pointer to the source file name
////*                  - line: assert_param error line source number
////* Output         : None
////* Return         : None
////*******************************************************************************/
////void assert_failed(uint8_t* file, uint32_t line)
////{
////    /* User can add his own implementation to report the file name and line number,
////    ex: SEGGER_RTT_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
////
////    /* Infinite loop */
////    while (1)
////    {}
////}
////#endif
