///******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
//* File Name          : I2C/Master_Polling/main.c
//* Author             : RF Application Team
//* Version            : V1.0.0
//* Date               : September-2015
//* Description        : Code demostrating the I2C master functionality
//********************************************************************************
//* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
//* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
//* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
//* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
//* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
//* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//*******************************************************************************/
//
///*Edited by Mona for Posture Monitor*/
//
//#include "I2C.h"
//#include "GPIO.h"
//#include "SEGGER_RTT.h"
//
//#include "bluenrg_x_device.h"
//#include <stdio.h>
//#include "BlueNRG1_conf.h"
//#include "SDK_EVAL_Config.h"
//
///* Private typedef -----------------------------------------------------------*/
//typedef enum {
//  I2C_OP_FAILED = 0,
//  I2C_OP_SUCCEED
//}
//I2C_AppOpStatus;
//
///* Private define ------------------------------------------------------------*/
///* I2C clock frequency */
//#define SDK_EVAL_I2C_CLK_SPEED  (10000)
//
//#define I2C_CLK_Pin		GPIO_Pin_7
//#define I2C_DAT_Pin 	GPIO_Pin_6
//
///* Private macro -------------------------------------------------------------*/\
///* Private variables ---------------------------------------------------------*/
///* Private functions ---------------------------------------------------------*/
//
///**
//* @brief  Process command code
//* @param  None
//* @retval None
//*/
////void processCommand(void)
////{
////  uint8_t command;
////  uint8_t led_status;
////
////  uint8_t pSend = 0;
////  uint8_t pRecieve = 0;
////
////  /* Infinite loop */
////
////    /* Loop until the UART Receive Data Register is not empty */
////    //while (UART_GetFlagStatus(UART_FLAG_RXFE) == SET);
////
////    /* Store the received byte in RxBuffer */
////    //command = UART_ReceiveData();
////
////	  SdkEvalI2CRead(&pRecieve, 0xA9, 0x0f, 1);
////
////	  if(pRecieve == 0x6C){
////    	 SEGGER_RTT_printf(0,"whoAmI Register read\n");
////    	 SEGGER_RTT_printf(0,"Register:%04x \n",pRecieve);
////       } else {
////    	   SEGGER_RTT_printf(0,"ERROR: whoAmI Register NOT read\n");
////    	   SEGGER_RTT_printf(0,"Register:%04x \n",pRecieve);
////       }
////  }
//
//
///**
//  * @brief  I2C Master initialization.
//  * @param  None
//  * @retval None
//  */
////void I2C_ConfigurationMaster(void)
////{
////	GPIO_InitType GPIO_InitStructure;
////	I2C_InitType I2C_InitStruct;
////
////  /* Enable I2C and GPIO clocks */
////  if((I2C_Type*)SDK_EVAL_I2C == I2C2) {
////    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C2 | CLOCK_PERIPH_GPIO, ENABLE);
////  }
////  else {
////    SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_I2C1 | CLOCK_PERIPH_GPIO, ENABLE);
////  }
//
////  /* Configure I2C pins */
////  GPIO_InitStructure.GPIO_Pin = I2C_CLK_Pin;
////  GPIO_InitStructure.GPIO_Mode = Serial0_Mode;
////  GPIO_InitStructure.GPIO_Pull = DISABLE;
////  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
////  GPIO_Init(&GPIO_InitStructure);
////
////  GPIO_InitStructure.GPIO_Pin = I2C_DAT_Pin;
////  GPIO_InitStructure.GPIO_Mode = Serial0_Mode;
////  GPIO_Init(&GPIO_InitStructure);
////
////  /* Configure I2C in master mode */
////  I2C_StructInit(&I2C_InitStruct);
////  I2C_InitStruct.I2C_OperatingMode = I2C_OperatingMode_Master;
////  I2C_InitStruct.I2C_ClockSpeed = SDK_EVAL_I2C_CLK_SPEED;
////  I2C_Init((I2C_Type*)SDK_EVAL_I2C, &I2C_InitStruct);
////
////  /* Clear all I2C pending interrupts */
////  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MSK);
////
////}
//
////void I2C_test (void){
////
////  /* I2C initialization */
////  I2C_ConfigurationMaster();
////
////  /* Infinite loop for process command */
////  processCommand();
////
////}
//
//
///**
//  * @brief  I2C Master communication.
//  * @param  uint8_t dev_address: I2C device address
//  * @param  uint8_t send_val: value to send
//  * @param  uint8_t get_val: value to get
//  * @retval I2C_OpStatus: I2C operation satus
//  */
////I2C_AppOpStatus I2C_Read_Reg(uint8_t dev_address, uint8_t send_val, uint8_t* get_val)
////{
////  I2C_TransactionType t;
////
////  /* Write I2C device address address */
////  t.Operation = I2C_Operation_Write;
////  t.Address = dev_address;
////  t.StartByte = I2C_StartByte_Disable;
////  t.AddressType = I2C_AddressType_7Bit;
////  t.StopCondition = I2C_StopCondition_Disable;
////  t.Length = 1;
//
//  I2C_FlushTx((I2C_Type*)SDK_EVAL_I2C);
//
//  /* wait until Flush finished */
//  while (I2C_WaitFlushTx((I2C_Type*)SDK_EVAL_I2C) == I2C_OP_ONGOING);
//
//  /* Write I2C device address address and put the send_val in TX FIFO */
//  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
//  I2C_FillTxFIFO((I2C_Type*)SDK_EVAL_I2C, send_val);
//
//  /* Check write */
//  do
//  {
//    if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
//      return I2C_OP_FAILED;
//
//  } while (I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS) == RESET);
//
//  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTDWS);
//
//  /* Prepare to read data */
//  t.Operation = I2C_Operation_Read;
//  t.Address = dev_address;
//  t.StartByte = I2C_StartByte_Disable;
//  t.AddressType = I2C_AddressType_7Bit;
//  t.StopCondition = I2C_StopCondition_Enable;
//  t.Length = 1;
//
//  I2C_BeginTransaction((I2C_Type*)SDK_EVAL_I2C, &t);
//
//  /* Check read */
//  do
//  {
//    if(I2C_OP_ABORTED == I2C_GetStatus((I2C_Type*)SDK_EVAL_I2C))
//      return I2C_OP_FAILED;
//
//  } while (RESET == I2C_GetITStatus((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD));
//
//  I2C_ClearITPendingBit((I2C_Type*)SDK_EVAL_I2C, I2C_IT_MTD | I2C_IT_MTDWS);
//
//  /* Get data */
//  *get_val = I2C_ReceiveData((I2C_Type*)SDK_EVAL_I2C);
//
//  return I2C_OP_SUCCEED;
//}
//
//
//#ifdef USE_FULL_ASSERT
//
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  */
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
//
//#endif
//
///**
//  * @}
//  */
//
///**
//  * @}
//  */
//
///**
//  * @}
//  */
//
///******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
//
//
