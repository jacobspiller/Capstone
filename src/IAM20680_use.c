/*
 * IAM20680_use.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Mona Dlikan
 */

//initialize IMU
//Collect raw accelerometer data
//Collect raw gyroscope data

#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "Quaternion.h"
#include "math.h"
#define M_PI		3.14159265358979323846

#include "IAM20680_use.h"
#include "Iam20680Driver_HL.h"
#include "Iam20680Product.h"
#include "Iam20680Defs.h"

#include "SEGGER_RTT.h"

//static lsm6dso32_pin_int1_route_t INT1Config;

#define IAM20680_ID		0xA9

volatile static uint8_t errorCounter = 0;

//static void IAM20680_CalibrateGyroscope(void);

extern void Error_Handler(void);

/* Low-level functions
 * The following functions return 0 if the I2C transaction was succesful; otherwise, 0.
 */
int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *bufp, uint16_t len){
	if(SdkEvalI2CWrite((uint8_t*) bufp, 0x68 >> 1, Reg, len) != SUCCESS){
		  return 1;
	  } else{
		  return 0;
	  }
}

int32_t platform_read(void *handle, uint8_t Reg, uint8_t *bufp, uint16_t len){
	 if(SdkEvalI2CRead((uint8_t*) bufp, 0x68 >> 1, Reg, len) != SUCCESS){
		 return 1;
	 } else{
		 return 0;
	 }
}

/* Initialization */

void IAM20680_Init(void){
	/* Initialize low-level driver */
//	IMUDriver.read_reg = platform_read;
//	IMUDriver.write_reg = platform_write;

	/* Errors in communication handler */
	errorCounter = 0;

	/* Check for the device ID */
	uint8_t i = 0;
	uint8_t regData = 0;

	/* Wait for the device to turn on */
	SdkDelayMs(40);

	errorCounter = inv_iam20680_get_who_am_i(&IMUDriver,&regData);

	if(regData == IAM20680_ID){
		/* Blink "CORRECT_ID" times to indicate correct communication with the device */
		while(i != CORRECT_ID){
			GPIO_WriteBit(GPIO_Pin_1, LED_ON);
			SdkDelayMs(250);
			i++;
		}
		i = 0;

		/* Stop blink to indicate that device will be configured next */
		GPIO_WriteBit(GPIO_Pin_1, LED_OFF);
		SdkDelayMs(500);
	} else{
		Error_Handler();
	}

	/****RESET THE DEVICE***/
	errorCounter = inv_iam20680_device_reset(&IMUDriver);

	/***GYROSCOPE***/
	errorCounter = inv_iam20680_wr_gyro_config_fs_sel(&IMUDriver, IAM20680_GYRO_CONFIG_FS_SEL_250dps);

	/***ACCELEROMETER***/
	errorCounter = inv_iam20680_wr_accel_config_accel_fs_sel(&IMUDriver, IAM20680_ACCEL_CONFIG_FS_SEL_2g );

	/* Activate accelerometer data-ready mode */
	errorCounter = inv_iam20680_enable_accel(&IMUDriver);
	errorCounter = inv_iam20680_enable_gyro(&IMUDriver);

	/* Blink "CONFIG_COMPLETE" times to indicate that the sensor was configured correctly */
	if(errorCounter == 0){
		while(i != CONFIG_COMPLETE){
			GPIO_WriteBit(GPIO_Pin_1, LED_ON);
			SdkDelayMs(100);
			i++;
		}
	} else{
		 Error_Handler();
	}
}

//
///* Data acquistion */
//void LSM6DSO32_ReadAccelerometer(float *pBuffer){
//	/* Combine raw data as 16 bits signed integers */
//	int16_t accRawSigned[3];
//	lsm6dso32_acceleration_raw_get(&IMUDriver, accRawSigned);
//	/* Convert to g and apply calibration equations */
//	if(!isIMUAccCalibrated){
//		pBuffer[0] = accRawSigned[0] * (0.122f * 0.001f);
//		pBuffer[1] = accRawSigned[1] * (0.122f * 0.001f);
//		pBuffer[2] = accRawSigned[2] * (0.122f * 0.001f);
//	} else {
//		pBuffer[0] = (((accRawSigned[0] * (0.122f * 0.001f)) * accSlope[0]) + accIntercept[0]);
//		pBuffer[1] = (((accRawSigned[1] * (0.122f * 0.001f)) * accSlope[1]) + accIntercept[1]);
//		pBuffer[2] = (((accRawSigned[2] * (0.122f * 0.001f)) * accSlope[2]) + accIntercept[2]);
////		/*convert to m/s^2 for testing with matlab*/
////		pBuffer[0] = (((accRawSigned[0] * (0.122f * 0.001f)) * accSlope[0]) + accIntercept[0])*9.81f;
////		pBuffer[1] = (((accRawSigned[1] * (0.122f * 0.001f)) * accSlope[1]) + accIntercept[1])*9.81f;
////		pBuffer[2] = (((accRawSigned[2] * (0.122f * 0.001f)) * accSlope[2]) + accIntercept[2])*9.81f;
//	}
//}
//
//void LSM6DSO32_ReadGyroscope(float *pBuffer){
//	/* Combine raw data as 16 bits signed integers */
//	int16_t gyroRawSigned[3];
//	lsm6dso32_angular_rate_raw_get(&IMUDriver, gyroRawSigned);
//	/* Convert data to rad/s and apply calibration offsets */
//	if(!isIMUGyroCalibrated){
//		pBuffer[0] = gyroRawSigned[0] * (8.75f * 0.001f);
//		pBuffer[1] = gyroRawSigned[1] * (8.75f * 0.001f);
//		pBuffer[2] = gyroRawSigned[2] * (8.75f * 0.001f);
//	} else{
//		pBuffer[0] = (gyroRawSigned[0] - gyroOffset[0]) * (8.75f * 0.001f) * (M_PI/180.0);
//		pBuffer[1] = (gyroRawSigned[1] - gyroOffset[1]) * (8.75f * 0.001f) * (M_PI/180.0);
//		pBuffer[2] = (gyroRawSigned[2] - gyroOffset[2]) * (8.75f * 0.001f) * (M_PI/180.0);
////		/*in deg/s if needed*/
////		pBuffer[0] = (gyroRawSigned[0] - gyroOffset[0]) * (8.75f * 0.001f);
////		pBuffer[1] = (gyroRawSigned[1] - gyroOffset[1]) * (8.75f * 0.001f);
////		pBuffer[2] = (gyroRawSigned[2] - gyroOffset[2]) * (8.75f * 0.001f);
//	}
//
//}
//
///* Interrupts management */
//void LSM6DSO32_dataReadyINT(uint8_t ENorDI){
//	errorCounter = 0;
//	if(ENorDI == ENABLE){
//		errorCounter = lsm6dso32_pin_int1_route_get(&IMUDriver, &INT1Config);
//		INT1Config.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
//		errorCounter = lsm6dso32_pin_int1_route_set(&IMUDriver, &INT1Config);
//		dataReadyINT = TRUE;
//	}
//	else if(ENorDI == DISABLE){
//		errorCounter = lsm6dso32_pin_int1_route_get(&IMUDriver, &INT1Config);
//		INT1Config.int1_ctrl.int1_drdy_xl = PROPERTY_DISABLE;
//		errorCounter = lsm6dso32_pin_int1_route_set(&IMUDriver, &INT1Config);
//		dataReadyINT = FALSE;
//	}
//	/* Check for errors in data transmission */
//	if(errorCounter != 0){
//		Error_Handler();
//	}
//}
//
///* Device calibration */
//BOOL LSM6DSO32_Calibrate(void){
//	errorCounter = 0;
//
//	/* Blink for "REST_SECONDS" seconds to indicate user to leave the device on a flat surface for calibration */
//	uint8_t i = 0;
//	while(i != REST_SECONDS){
//		GPIO_ToggleBits(GPIO_Pin_1);
//		SdkDelayMs(1000);
//		i++;
//	}
//	i = 0;
//
//	/*  Indicate that the device calibration will start */
//	GPIO_WriteBit(GPIO_Pin_1, Bit_SET);
//	SdkDelayMs(250);
//
//	/* Activate data-ready interrupts on Pin INT1 */
//	LSM6DSO32_dataReadyINT(ENABLE);
//
//	/* Wait for data-ready signal to stabilize */
//	SdkDelayMs(10);
//
//	/* Calibrate gyroscope */
//	LSM6DSO32_CalibrateGyroscope();
//
//	/* Deactivate data-ready interrupts on pin INT1 */
//	LSM6DSO32_dataReadyINT(DISABLE);
//
//	/* Wait for data-ready signal to stabilize */
//	SdkDelayMs(10);
//
//	/* Turn off LED */
//	GPIO_WriteBit(GPIO_Pin_1, Bit_RESET);
//
//	/* Check if the device was successfully calibrated */
//	if(errorCounter != 0){
//		return FALSE;
//	} else{
//		return TRUE;
//	}
//}
//
//static void LSM6DSO32_CalibrateGyroscope(void){
//	/* Calibration variables and buffers */
//	int16_t calibrationBufferX[BUFFERSIZE];
//	int16_t calibrationBufferY[BUFFERSIZE];
//	int16_t calibrationBufferZ[BUFFERSIZE];
//
//	/* Clear calibrationAverage buffer */
//	int calibrationAverage[3];
//	memset(calibrationAverage,0,sizeof(calibrationAverage));
//
//	/* Register offset */
//	int16_t registerOffset[3];
//
//	uint8_t i = 0;
//
//	/* Read raw data */
//	while(i != BUFFERSIZE){
//		if(dataReadyFlag){
//			int16_t rawDataBuffer[3];
//			lsm6dso32_angular_rate_raw_get(&IMUDriver, rawDataBuffer);
//			calibrationBufferX[i] = rawDataBuffer[0];
//			calibrationBufferY[i] = rawDataBuffer[1];
//			calibrationBufferZ[i] = rawDataBuffer[2];
//			i++;
//			dataReadyFlag = FALSE;
//		}
//	}
//	i = 0;
//
//	/* Calculate data average */
//	for(i = 0;i < BUFFERSIZE; i++){
//		calibrationAverage[0] += calibrationBufferX[i];
//		calibrationAverage[1] += calibrationBufferY[i];
//		calibrationAverage[2] += calibrationBufferZ[i];
//	}
//
//	/* Gyroscope offset */
//	for(i = 0; i < 3; i++){
//		gyroOffset[i] = (uint16_t)(calibrationAverage[i] / BUFFERSIZE);
//	}
//}
