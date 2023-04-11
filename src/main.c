
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
#include "I2C.h"
#include "IAM20680_use.h"
#include "Quaternion.h"
#include "Vector.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_NEW_SERIALPORT_VERSION_STRING "2.0.0"
#define SERVER 1

#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

BOOL volatile longTermMode = FALSE;
volatile uint32_t currentAddress = 0;
volatile uint32_t lSystickCounter = 0;

volatile BOOL isIMUAccCalibrated = FALSE;
volatile BOOL isIMUGyroCalibrated = FALSE;

float accReadings[3];
float gyroReadings[3];

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

	int ret;
	int rc = 0;
	inv_bool_t isSPI = SERIF_TYPE_SPI;

  /* System initialization function */
  SystemInit();

  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();

  // SdkEvalComUartInit(UART_BAUDRATE);
  LED_Init();

  /*Initialization*/
  longTermMode = FALSE;
  /* Configure I2C as master mode */
  I2C_ConfigurationMaster();
  /* GPIO configuration */
  GPIO_Configuration();
  /*LED initialization*/
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

   /*IMU*/
   /* IAM20680 Init and Calibration*/
   /* Initialize SPI/I2C Interface */
   interface_initialize();

   SetupInvDevice(idd_io_hal_read_reg, idd_io_hal_write_reg, 0);
   //check_rc(rc, "error while setting up device");

   	/* Configure Device */

   	inv_dev_config_t dev_config;
   	dev_config.enable_accel = (uint8_t)ENABLE_ACCEL;
   	dev_config.enable_gyro = (uint8_t)ENABLE_GYRO;
   	//dev_config.set_low_noise_mode = (uint8_t)LOW_POWER_MODE;
   	dev_config.acc_fsr_g = FSR_ACC_G;
   	dev_config.gyr_fsr_dps = FSR_GYR_DPS;
   	//dev_config.odr_us = ODR_US;

   	ConfigureInvDevice(&dev_config);
   	//check_rc(rc, "error while configuring device");

       	  //IAM20680_Init();
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

   while(1) {

       /* BlueNRG-1,2 stack tick */
       BTLE_StackTick();

       /* Application tick */
       APP_Tick();

       joint_set(260,0,0);

       /*Collect raw data from IMU*/
       	  	  if(dataReadyFlag){
       	  		  IAM20680_ReadGyroscope(gyroReadings);
       	  		  IAM20680_ReadAccelerometer(accReadings);

       	  		  /*Raw Data display*/
       	  		  SEGGER_RTT_printf(0, "%d.%3d    %d.%3d    %d.%3d    %d.%3d    %d.%3d   %d.%3d   \r\n",
       	  				  PRINT_INT(accReadings[0]), PRINT_FLOAT(accReadings[0]),
       	  				  	  PRINT_INT(accReadings[1]), PRINT_FLOAT(accReadings[1]),
       	  						  PRINT_INT(accReadings[2]), PRINT_FLOAT(accReadings[2]),
       	  						  	  PRINT_INT(gyroReadings[0]), PRINT_FLOAT(gyroReadings[0]),
       	  							  	  PRINT_INT(gyroReadings[1]), PRINT_FLOAT(gyroReadings[1]),
       	  								  	  PRINT_INT(gyroReadings[2]), PRINT_FLOAT(gyroReadings[2]));

       	  		  /*Apply Quaternions to raw data to convert to Euler Angles*/
       	  		  float accel_x = accReadings[0];
       	  		  float accel_y = accReadings[1];
       	  		  float accel_z = accReadings[2];
       	  		  float gyro_x = gyroReadings[0];
       	  		  float gyro_y = gyroReadings[1];
       	  		  float gyro_z = gyroReadings[2];

       	  		  float* roll = 0;
       	  		  float* pitch = 0;
       	  		  float* yaw = 0;

       	  		  calculateEulerAngles(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw);
       	  		  SEGGER_RTT_printf (0,"Pitch Angle: %3f\n",pitch);
       	  		  SEGGER_RTT_printf (0,"Roll Angle: %3f\n",roll);
       	  		  SEGGER_RTT_printf (0,"Yaw Angle: %3f\n",yaw);
       	  	  }
   	   }
}

void SdkDelayMs(volatile uint32_t lTimeMs)
{
  uint32_t nWaitPeriod = ~lSystickCounter;

  if(nWaitPeriod<lTimeMs)
  {
    while( lSystickCounter != 0xFFFFFFFF);
    nWaitPeriod = lTimeMs-nWaitPeriod;
  }
  else
    nWaitPeriod = lTimeMs+ ~nWaitPeriod;

  while( lSystickCounter != nWaitPeriod ) ;

}
