
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
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "app_state.h"
#include "serial_port.h"
#include "SDK_EVAL_Config.h"
#include "SerialPort_config.h"
#include "OTA_btl.h"
#include "bluenrg_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "GPIO.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_conf.h"
#include "BlueNRG1_mft.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_NEW_SERIALPORT_VERSION_STRING "2.0.0"
#define SERVER 1


/**
  * @brief  Main program code
  * @param  None
  * @retval None
  */
int main(void)
{
	int ret;
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t z = 0;
  /* System initialization function */
  SystemInit();

  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();

  SdkEvalComUartInit(UART_BAUDRATE);
  LED_Init();
  SEGGER_RTT_printf (0,"1\n");

   SEGGER_RTT_printf (0,"starting bluetooth\n");
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
	  SEGGER_RTT_printf(0,"Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
  }
  SEGGER_RTT_printf (0,"%x\n",ret);

  SEGGER_RTT_printf(0,"BlueNRG-1,2 BLE Serial Port Master & Slave Application (version: %s)\r\n",BLE_NEW_SERIALPORT_VERSION_STRING);

    /* Init Serial Port Device */
   ret = SerialPort_DeviceInit();
   if (ret != BLE_STATUS_SUCCESS) {
     SEGGER_RTT_printf(0,"SerialPort_DeviceInit()--> Failed 0x%02x\r\n", ret);
     while(1);
   }
	/* GPIO configuration */

   while(1) {
	   GPIO_Configuration(x,y,z);
       /* BlueNRG-1,2 stack tick */
       BTLE_StackTick();

       /* Application tick */
       APP_Tick();
       x = return_xangle();
       y = return_yangle();
       z = return_zangle();
       SEGGER_RTT_printf(0,"%d\n",y);
       joint_set(x,y,0);
   }


}
