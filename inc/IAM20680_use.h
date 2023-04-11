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
//#include "Iam20680Driver_HL.h"
#include "Iam20680Product.h"
#include "Iam20680Defs.h"
#include "BlueNRG1_conf.h"
#include "InvBool.h"

int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);

void interface_initialize(void);

typedef struct inv_dev_config {
	uint8_t enable_accel;       // 1: Enable Accel, 0: Disable Accel
	uint8_t enable_gyro;        // 1: Enable Gyro, 0: Disable Gyro
	uint8_t set_low_noise_mode; // 1: Set the device in low noise mode, 0: Set the device in low power mode
	int32_t acc_fsr_g;          // Accel Full Scale Range in g
	int32_t gyr_fsr_dps;        // Gyro Full Scale Range in dps
	uint32_t odr_us;            // Output Data Rate in micro second
} inv_dev_config_t;

#define ENABLE_ACCEL  			1
#define ENABLE_GYRO   			1

#define LOW_NOISE_MODE			1
#define LOW_POWER_MODE			0

#define ODR_US           		20000	/* Default odr set to 20ms (50Hz) */
#define FSR_ACC_G        		4      	/* +/- 4g */
#define FSR_GYR_DPS      		2000   	/* +/- 2000dps */

#define SERIF_TYPE_SPI              0
#define SERIF_TYPE_I2C              1

//#include "Quaternion.h"
//#include "Vector.h"
//
#define BUFFERSIZE 			90 /* This value must be a multiple of 3 */
#define CORRECT_ID			10
#define CONFIG_COMPLETE		4
#define REST_SECONDS		4
#define DEBOUNCE_STEPS		10
//
//#define ACCELEROMETER		0
//#define GYROSCOPE			1
//
//#define DISABLE				0
//#define ENABLE				1

//#define ACCEL_LOWER_LIMIT 6144
//#define ACCEL_UPPER_LIMIT 10240

volatile BOOL dataReadyINT;
volatile BOOL dataReadyFlag;



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
int SetupInvDevice(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),
				   int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len),
				   inv_bool_t isSPI);
int ConfigureInvDevice(inv_dev_config_t* device_config);

///* Data acquisition */
int32_t IAM20680_acceleration_raw_get(struct inv_iam20680 * s , int16_t *val);
int32_t IAM20680_angular_rate_raw_get(struct inv_iam20680 * s , int16_t *val);

void IAM20680_ReadAccelerometer(float *pBuffer);
void IAM20680_ReadGyroscope(float *pBuffer);

/* Device calibration */
BOOL IAM20680_Calibrate(void);

#endif /* INC_IAM20680_USE_H_ */
