//heavily update when put in the code for this IMU
#ifndef USER_INC_IAM20680_H_
#define USER_INC_IAM20680_H_


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

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

//stmdev_ctx_t IMUDriver;

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
void LSM6DSO32_Init(void);

/* Data acquisition */
void LSM6DSO32_ReadAccelerometer(float *pBuffer);
void LSM6DSO32_ReadGyroscope(float *pBuffer);

/* Interrupts management */
void LSM6DSO32_dataReadyINT(uint8_t ENorDI);

/* Device calibration */
BOOL LSM6DSO32_Calibrate(void);

//Quaternion quaternion_from_accelerometer(float ax, float ay, float az);
//Quaternion quaternion_from_gyro(float wx, float wy, float wz, float time);
//float fusion_coeffecient(vector_ijk virtual_gravity, vector_ijk sensor_gravity);
//vector_ijk sensor_gravity_normalized(int16_t ax, int16_t ay, int16_t az);
//vector_ijk fuse_vector(vector_ijk virtual_gravity, vector_ijk sensor_gravity);
//vector_ijk update_gravity_vector(vector_ijk gravity_vector,float wx,float wy,float wz,float delta);
//vector_ijk update_fused_vector(vector_ijk fused_vector, int16_t ax, int16_t ay, int16_t az,float wx,float wy,float wz,float delta);


#endif /* USER_INC_IAM20680_H_ */
