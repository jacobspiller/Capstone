/*
 * IAM20680_use.h
 *
 *  Created on: Apr 10, 2023
 *      Author: Mona Dlikan
 */

#ifndef INC_IAM20680_USE_H_
#define INC_IAM20680_USE_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "IAM20680_use.h"
#include "Iam20680Driver_HL.h"
#include "Iam20680Product.h"
#include "Iam20680Defs.h"
#include "BlueNRG1_conf.h"

#include "Quaternion.h"
#include "Vector.h"

#define BUFFERSIZE 			90 /* This value must be a multiple of 3 */
#define CORRECT_ID			10
#define CONFIG_COMPLETE		4
#define REST_SECONDS		4
#define DEBOUNCE_STEPS		10

#define ACCELEROMETER		0
#define GYROSCOPE			1

#define DISABLE				0
#define ENABLE				1

//#define ACCEL_LOWER_LIMIT 6144
//#define ACCEL_UPPER_LIMIT 10240

volatile BOOL dataReadyINT;
volatile BOOL dataReadyFlag;

inv_iam20680_t IMUDriver;

extern volatile BOOL isIMUAccCalibrated;
volatile float accSlope[3];
volatile float accIntercept[3];

volatile int16_t gyroOffset[3];
extern volatile BOOL isIMUGyroCalibrated;

extern void SdkDelayMs(volatile uint32_t lTimeMs);

/* Low-level functions */
int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t Reg, uint8_t *bufp, uint16_t len);

/* Initialization */
void IAM20680_Init(void);

///* Data acquisition */
//void LSM6DSO32_ReadAccelerometer(float *pBuffer);
//void LSM6DSO32_ReadGyroscope(float *pBuffer);
//
///* Interrupts management */
//void LSM6DSO32_dataReadyINT(uint8_t ENorDI);
//
///* Device calibration */
//BOOL LSM6DSO32_Calibrate(void);

#endif /* INC_IAM20680_USE_H_ */
