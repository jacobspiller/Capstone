/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
* File Name          : I2C/Master_Polling/main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : September-2015
* Description        : Code demostrating the I2C master functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/*Edited by Mona for Posture Monitor*/

#include "I2C.h"
#include "GPIO.h"
#include "SEGGER_RTT.h"

#include "bluenrg_x_device.h"
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "SEGGER_RTT.h"
#include "SEGGER_RTT_conf.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  I2C_OP_FAILED = 0,
  I2C_OP_SUCCEED
}
I2C_AppOpStatus;

/* Private define ------------------------------------------------------------*/
/* I2C clock frequency */
#define SDK_EVAL_I2C_CLK_SPEED  (10000)

#define I2C_CLK_Pin		GPIO_Pin_7
#define I2C_DAT_Pin 	GPIO_Pin_6

/* Private macro -------------------------------------------------------------*/\
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Process command code
* @param  None
* @retval None
*/
void processCommand(void)
{
  uint8_t command;
  uint8_t led_status;

  uint8_t pSend = 0;
  uint8_t pRecieve = 0;

  /* Infinite loop */

    /* Loop until the UART Receive Data Register is not empty */
    //while (UART_GetFlagStatus(UART_FLAG_RXFE) == SET);

    /* Store the received byte in RxBuffer */
    //command = UART_ReceiveData();

	  SdkEvalI2CRead(&pRecieve, 0x68, 0x75, 1);

	  if(pRecieve == 0xA9){
    	 SEGGER_RTT_printf(0,"whoAmI Register read\n");
    	 SEGGER_RTT_printf(0,"Register:%04x \n",pRecieve);
       } else {
    	   SEGGER_RTT_printf(0,"ERROR: whoAmI Register NOT read\n");
    	   SEGGER_RTT_printf(0,"Register:%04x \n",pRecieve);
       }
  }


/**
  * @brief  I2C Master initialization.
  * @param  None
  * @retval None
  */
void I2C_ConfigurationMaster(void)
{
	GPIO_InitType GPIO_InitStructure;
	I2C_InitType I2C_InitStruct;

  /* Enable I2C and GPIO clocks */
  if((I2C_Type*)SDK_EVAL_I2C == I2C2) {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C2 | CLOCK_PERIPH_GPIO, ENABLE);
  }
  else {
    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C1 | CLOCK_PERIPH_GPIO, ENABLE);
  }

  /* Configure I2C pins */
  GPIO_InitStructure.GPIO_Pin = I2C_CLK_Pin;
  GPIO_InitStructure.GPIO_Mode = Serial0_Mode;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = I2C_DAT_Pin;
  GPIO_InitStructure.GPIO_Mode = Serial0_Mode;
  GPIO_Init(&GPIO_InitStructure);

  /* Configure I2C in master mode */
  I2C_StructInit(&I2C_InitStruct);
  I2C_InitStruct.I2C_OperatingMode = I2C_OperatingMode_Master;
  I2C_InitStruct.I2C_ClockSpeed = SDK_EVAL_I2C_CLK_SPEED;
  I2C_Init((I2C_Type*)SDK_EVAL_I2C, &I2C_InitStruct);

  /* Clear all I2C pending interrupts */
  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MSK);

}

void I2C_test (void){

  /* I2C initialization */
  I2C_ConfigurationMaster();

  /* Infinite loop for process command */
  processCommand();

}


/**
  * @brief  I2C Master communication.
  * @param  uint8_t dev_address: I2C device address
  * @param  uint8_t send_val: value to send
  * @param  uint8_t get_val: value to get
  * @retval I2C_OpStatus: I2C operation satus
  */
I2C_AppOpStatus I2C_Read_Reg(uint8_t dev_address, uint8_t send_val, uint8_t* get_val)
{
  I2C_TransactionType t;

  /* Write I2C device address address */
  t.Operation = I2C_Operation_Write;
  t.Address = dev_address;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Disable;
  t.Length = 1;

  I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);

  /* wait until Flush finished */
  while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);

  /* Write I2C device address address and put the send_val in TX FIFO */
  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
  I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, send_val);

  /* Check write */
  do
  {
    if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
      return I2C_OP_FAILED;

  } while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS) == RESET);

  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS);

  /* Prepare to read data */
  t.Operation = I2C_Operation_Read;
  t.Address = dev_address;
  t.StartByte = I2C_StartByte_Disable;
  t.AddressType = I2C_AddressType_7Bit;
  t.StopCondition = I2C_StopCondition_Enable;
  t.Length = 1;

  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);

  /* Check read */
  do
  {
    if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
      return I2C_OP_FAILED;

  } while (RESET == I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD));

  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);

  /* Get data */
  *get_val = I2C_ReceiveData((I2C_Type*)SDK_EVAL_I2C);

  return I2C_OP_SUCCEED;
}


#ifdef USE_FULL_ASSERT

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

int main(void){
	uint8_t ret;
	SEGGER_RTT_Init();
	SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

	  /* System Init */
	  SystemInit();

	  /* Identify BlueNRG1 platform */
	  SdkEvalIdentification();

	  /* Configure I/O communication channel */
	  SdkEvalComUartInit(UART_BAUDRATE);
	  /* Configure I2C as master mode */
	  I2C_ConfigurationMaster();
	  I2C_test();
}

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






///******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
//* File Name          : SensorDemo_main.c
//* Author             : AMS - RF Application team
//* Version            : V1.2.0
//* Date               : 03-December-2018
//* Description        : BlueNRG-1-2 Sensor Demo main file
//********************************************************************************
//* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
//* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
//* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
//* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
//* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//*******************************************************************************/
///** @addtogroup BlueNRG1_demonstrations_applications
// * BlueNRG-1,2 SensorDemo \see SensorDemo_main.c for documentation.
// *
// *@{
// */
//
///** @} */
///** \cond DOXYGEN_SHOULD_SKIP_THIS
// */
///* Includes ------------------------------------------------------------------*/
//#include <stdio.h>
//#include "BlueNRG1_it.h"
//#include "BlueNRG1_conf.h"
//#include "ble_const.h"
//#include "bluenrg1_stack.h"
//#include "SDK_EVAL_Config.h"
//#include "sleep.h"
//#include "sensor.h"
//#include "SensorDemo_config.h"
//#include "OTA_btl.h"
//#include "gatt_db.h"
//#include "SEGGER_RTT.h"
//#include "SEGGER_RTT_conf.h"
//
//
///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
//#ifndef DEBUG
//#define DEBUG 0
//#endif
//
//#if DEBUG
//#include <stdio.h>
////#define PRINTF(...) printf(__VA_ARGS__)
//#else
//#define PRINTF(...)
//#endif
//
//#define BLE_SENSOR_VERSION_STRING "1.0.0"
//
//
//
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//
///* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
//uint8_t Services_Max_Attribute_Records[NUMBER_OF_APPLICATION_SERVICES] = {MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2};
//
///* Private function prototypes -----------------------------------------------*/
///* Private functions ---------------------------------------------------------*/
//
//int main(void)
//{
//  uint8_t ret;
//SEGGER_RTT_Init();
//SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
//
//  /* System Init */
//  SystemInit();
//
//  /* Identify BlueNRG1 platform */
//  SdkEvalIdentification();
//
//  /* Configure I/O communication channel */
//  SdkEvalComUartInit(UART_BAUDRATE);
//
//  	GPIO_InitType GPIO_InitStructure;
//  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
//  	GPIO_InitStructure.GPIO_Pull = ENABLE;
//  	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
//  	GPIO_Init(&GPIO_InitStructure);
//
//  	//InitGPIO_Output(LED_PIN);
//  	//LED_Configuration();
////  	ToggleLED();
//
//
//  /* BlueNRG-1,2 stack init */
////  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
////  if (ret != BLE_STATUS_SUCCESS) {
////	  SEGGER_RTT_printf(0,"Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
//    while(1);
////  }
//
////  /* Application demo Led Init */
////  SdkEvalLedInit(LED1); //Activity led
////  SdkEvalLedInit(LED3); //Error led
////  SdkEvalLedOn(LED1);
////  SdkEvalLedOff(LED3);
//	GPIO_WriteBit(GPIO_Pin_1, LED_ON);
//
//  SEGGER_RTT_printf(0,"BlueNRG-1,2 BLE Sensor Demo Application (version: %s)\r\n", BLE_SENSOR_VERSION_STRING);
//
//#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
//  /* Initialize the button: to be done before Sensor_DeviceInit for avoiding to
//     overwrite pressure/temperature sensor IO configuration when using BUTTON_2 (IO5) */
//  SdkEvalPushButtonInit(USER_BUTTON);
//#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
//
//  /* Sensor Device Init */
//  ret = Sensor_DeviceInit();
//  if (ret != BLE_STATUS_SUCCESS) {
//    SdkEvalLedOn(LED3);
//    while(1);
//  }
//
//
//  while(1)
//  {
//    /* BLE Stack Tick */
//    BTLE_StackTick();
//
//    /* Application Tick */
//    APP_Tick();
//
//    /* Power Save management */
//    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
//
//#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
//    /* Check if the OTA firmware upgrade session has been completed */
//    if (OTA_Tick() == 1)
//    {
//      /* Jump to the new application */
//      OTA_Jump_To_New_Application();
//    }
//#endif /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */
//
//#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
//    if (SdkEvalPushButtonGetState(USER_BUTTON) == RESET)
//    {
//      OTA_Jump_To_Service_Manager_Application();
//    }
//#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
//  }/* while (1) */
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
///**
//  * This event is generated to report firmware error informations.
//  * FW_Error_Type possible values:
//  * Values:
//  - 0x01: L2CAP recombination failure
//  - 0x02: GATT unexpected response
//  - 0x03: GATT unexpected request
//    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect.
//*/
//void aci_hal_fw_error_event(uint8_t FW_Error_Type,
//                            uint8_t Data_Length,
//                            uint8_t Data[])
//{
//  if (FW_Error_Type <= 0x03)
//  {
//    uint16_t connHandle;
//
//    /* Data field is the connection handle where error has occurred */
//    connHandle = LE_TO_HOST_16(Data);
//
//    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER);
//  }
//}
//
///****************** BlueNRG-1,2 Sleep Management Callback ********************************/
//
//SleepModes App_SleepMode_Check(SleepModes sleepMode)
//{
//  if(request_free_fall_notify || SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
//    return SLEEPMODE_RUNNING;
//
//  return SLEEPMODE_NOTIMER;
//}
//
///***************************************************************************************/
//
//#ifdef USE_FULL_ASSERT
///*******************************************************************************
//* Function Name  : assert_failed
//* Description    : Reports the name of the source file and the source line number
//*                  where the assert_param error has occurred.
//* Input          : - file: pointer to the source file name
//*                  - line: assert_param error line source number
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void assert_failed(uint8_t* file, uint32_t line)
//{
//    /* User can add his own implementation to report the file name and line number,
//    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//    /* Infinite loop */
//    while (1)
//    {}
//}
//#endif
//
///******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
///** \endcond
// */
//
//
////  /******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
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
////#include <stdlib.h>
////#include "BlueNRG1_it.h"
////#include "BlueNRG1_conf.h"
////#include "ble_const.h"
////#include "bluenrg1_stack.h"
////#include "SDK_EVAL_Config.h"
////#include "sleep.h"
////#include "SEGGER_RTT.h"
////#include "SEGGER_RTT_conf.h"
////	SEGGER_RTT_Init();
////	SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
////#include "clock.h"
////
//////#include "sensor.h"
//////#include "SensorDemo_config.h"
////#include "OTA_btl.h"
//////#include "gatt_db.h"
////#include "BlueNRG1_gpio.h"
//////#include "GPIO.h"
//////#include "SEGGER_RTT.h"
////
//////#include "I2C.h"
//////#include "GPIO.h"
//////#include "SEGGER_RTT.h"
////
////#include "bluenrg_x_device.h"
////#include <stdio.h>
////#include "BlueNRG1_conf.h"
////#include "SDK_EVAL_Config.h"
//
////
/////* Private typedef -----------------------------------------------------------*/
/////* Private define ------------------------------------------------------------*/
////#define LED_PIN GPIO_Pin_0
////
////#define PRINT_INT(x)    ((int)(x))
////#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))
//////#define BLE_SENSOR_VERSION_STRING "1.0.0"
////
/////* Private macro -------------------------------------------------------------*/
/////* Private variables ---------------------------------------------------------*/
////
/////* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
/////* Private function prototypes -----------------------------------------------*/
/////* Private functions ---------------------------------------------------------*/
////
////int main(void){
////	uint8_t ret;
////  /*SEGGER Init*/
////	SEGGER_RTT_Init();
////	SEGGER_RTT_ConfigUpBuffer(0,NULL,NULL,0,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
////
////  /*STM Initialization*/
////  	/* System Init */
////  	SystemInit();
////  	/* Identify BlueNRG1 platform */
////  	SdkEvalIdentification();
////  	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);
////  	/* Configure I/O communication channel */
////  	SdkEvalComUartInit(UART_BAUDRATE);
////
////  	/*SysTick Initialization 1ms*/
////  	Clock_Init();
////	SysTick_Config(SYST_CLOCK/1000 - 1);
//////	SEGGER_RTT_printf(0,"hello\n");
////
////	GPIO_InitType GPIO_InitStructure;
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
////	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
////	GPIO_InitStructure.GPIO_Pull = ENABLE;
////	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
////	GPIO_Init(&GPIO_InitStructure);
////
////	//InitGPIO_Output(LED_PIN);
////	//LED_Configuration();
////	//ToggleLED();
////	GPIO_WriteBit(GPIO_Pin_1, LED_ON);
////	while(1){
////		SEGGER_RTT_printf(0,"GPIO Test: OK \n");
////	};
////}
