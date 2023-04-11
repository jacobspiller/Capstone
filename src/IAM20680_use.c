/*
 * IAM20680_use.c
 *
 *  Created on: Apr 10, 2023
 *      Author: Mona Dlikan
 */

#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
//#include "Quaternion.h"
#include "math.h"
#define M_PI		3.14159265358979323846

#include "IAM20680_use.h"
#include "Iam20680Driver_HL.h"
#include "Iam20680Product.h"
#include "Iam20680Defs.h"
#include "GPIO.h"
#include "I2C.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_conf.h"

#define IAM20680_ID		0xA9
uint8_t	I2C_Address = IAM20680_ID;

//volatile static uint8_t errorCounter = 0;
static inv_iam20680_t IMUDriver;

//static void IAM20680_CalibrateGyroscope(void);

//extern void Error_Handler(void);

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

int SetupInvDevice(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),
				   int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len),
				   inv_bool_t isSPI){
	int rc = 0;
		uint8_t who_am_i;

		/* Initialize iam20680 serif structure */
		struct inv_iam20680_serif iam20680_serif;

		iam20680_serif.context   = 0; /* no need */
		iam20680_serif.read_reg  = (*read_reg);
		iam20680_serif.write_reg = (*write_reg);
		iam20680_serif.is_spi    = isSPI;
		if(isSPI) {
			/* Init SPI communication: SPI1 - SCK(PA5) / MISO(PA6) / MOSI(PA7) / CS(PB6) */
			iam20680_serif.max_read  = 1024*32; /* maximum number of bytes allowed per serial read */
			iam20680_serif.max_write = 1024*32; /* maximum number of bytes allowed per serial write */
			//INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through SPI");
		}else {
			/* Init I2C communication: I2C1 - SCL(PB8) / SDA(PB9) */
			iam20680_serif.max_read  = 64; /* maximum number of bytes allowed per serial read */
			iam20680_serif.max_write = 64; /* maximum number of bytes allowed per serial write */
			//INV_MSG(INV_MSG_LEVEL_INFO, "Opening serial interface through I2C");
		}

		/* Reset iam20680 driver states */
		memset(&IMUDriver, 0, sizeof(IMUDriver));
		IMUDriver.serif = iam20680_serif;

		/* Check WHOAMI */
//		rc = inv_iam20680_get_who_am_i(&IMUDriver, &who_am_i);
//		#if (SERIF_TYPE_I2C == 1)
//		if((rc != INV_ERROR_SUCCESS) || (who_am_i != EXPECTED_WHOAMI)) {
//			if(!isSPI) {
//				/* Check i2c bus stuck and clear the bus */
//				twi_clear_bus();
//			}
//
//			/* Retry who_am_i check */
//			rc = inv_iam20680_get_who_am_i(&IMUDriver, &who_am_i);
//		}
//		#endif
//
//		if(rc != INV_ERROR_SUCCESS)
//			return rc;
//
//		if(who_am_i != EXPECTED_WHOAMI) {
//			//INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected 0x%02x.", who_am_i, EXPECTED_WHOAMI);
//		}

		/* Initialize device */
		rc = inv_iam20680_init(&IMUDriver);

		return rc;
    }

int ConfigureInvDevice(inv_dev_config_t* device_config)
{
	int rc = 0;
	/* Set full scale range */
		rc |= inv_iam20680_set_gyro_fsr(&IMUDriver, device_config->gyr_fsr_dps);
		rc |= inv_iam20680_set_accel_fsr(&IMUDriver, device_config->acc_fsr_g);

		/* Enable Accel and Gyro axes */
		if(device_config->enable_accel) {
			rc |= inv_iam20680_enable_accel(&IMUDriver);
			if (!(device_config->set_low_noise_mode) && !(device_config->enable_gyro)) {
				rc |= inv_iam20680_wr_lp_mode_cfg_set_lp_accel_odr(&IMUDriver, (double)1000000.0/device_config->odr_us);
				rc |= inv_iam20680_wr_accel_config2_a_dlpf_cfg(&IMUDriver, IAM20680_ACCEL_CONFIG2_A_DLPF_CFG_420);
				rc |= inv_iam20680_set_accel_low_power_mode(&IMUDriver);
			}
		}

		if(device_config->enable_gyro) {
			rc |= inv_iam20680_enable_gyro(&IMUDriver);
		}
		return rc;
}

int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen) {
	(void)context;

#if (SERIF_TYPE_SPI == 1)
	return spi_master_read_register(NULL, reg, rbuffer, rlen);
#elif (SERIF_TYPE_I2C == 1)
	//return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
	return platform_read(context, reg, rbuffer, rlen);
#endif
}

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen) {
	(void)context;

#if (SERIF_TYPE_SPI == 1)
	return spi_master_write_register(NULL, reg, wbuffer, wlen);
#elif (SERIF_TYPE_I2C == 1)
	return platform_write(context, reg, wbuffer, wlen);
#endif
}

void interface_initialize(void) {

#if (SERIF_TYPE_SPI == 1)

	spi_master_initialize();

#elif (SERIF_TYPE_I2C == 1)
	/*
	 * Default IAM20680 (Slave) Address = 0x68 (pin AD0 is logic low)
	 * Change Slave Address to 0x69 (pin AD0 is logic high)
	 */
//#if ((TDK_BOARD_INBUILT_SENSOR == 1) && (TDK_BOARD_REVISION == 1))
//	I2C_Address = IAM_I2C_ADDR_AD0_HIGH;
//#endif
	I2C_ConfigurationMaster();
#endif
}
//
//void IAM20680_Init(void){
//	/* Initialize low-level driver */
	//IMUDriver.read_reg = platform_read;
	// IMUDriver.write_reg = platform_write;
//
//	/* Errors in communication handler */
//	uint8_t errorCounter = 0;
//
//	/* Check for the device ID */
//	uint8_t i = 0;
//	uint8_t regData;
//
//	/* Wait for the device to turn on */
//	SdkDelayMs(40);
//
//	errorCounter = inv_iam20680_get_who_am_i(&IMUDriver,&regData);
//
//	if(regData == IAM20680_ID){
//		/* Blink "CORRECT_ID" times to indicate correct communication with the device */
//		while(i != CORRECT_ID){
////			GPIO_WriteBit(BL, LED_ON);
//			SdkDelayMs(250);
////			GPIO_WriteBit(GPIO_Pin_1, LED_OFF);
//			i++;
//		}
//		i = 0;
//
//		/* Stop blink to indicate that device will be configured next */
////		GPIO_WriteBit(GPIO_Pin_1, LED_OFF);
//		SdkDelayMs(500);
//	} else{
//		SEGGER_RTT_printf(0,"error! monas fault\n");
//	}
//
//	/****RESET THE DEVICE***/
////	errorCounter = inv_iam20680_device_reset(&IMUDriver);
//
//	/***GYROSCOPE***/
////	errorCounter = inv_iam20680_wr_gyro_config_fs_sel(&IMUDriver, IAM20680_GYRO_CONFIG_FS_SEL_250dps);
//
//	/***ACCELEROMETER***/
////	errorCounter = inv_iam20680_wr_accel_config_accel_fs_sel(&IMUDriver, IAM20680_ACCEL_CONFIG_FS_SEL_2g );
//
//	/* Activate accelerometer data-ready mode */
////	errorCounter = inv_iam20680_enable_accel(&IMUDriver);
////	errorCounter = inv_iam20680_enable_gyro(&IMUDriver);
//
//	/* Blink "CONFIG_COMPLETE" times to indicate that the sensor was configured correctly */
//	if(errorCounter == 0){
//		while(i != CONFIG_COMPLETE){
////			GPIO_WriteBit(GPIO_Pin_1, LED_ON);
//			SdkDelayMs(100);
//			i++;
//		}
//	} else{
//		SEGGER_RTT_printf(0,"error! monas fault\n");
////		 Error_Handler();
//	}
//
//}

int32_t IAM20680_acceleration_raw_get(struct inv_iam20680 * s , int16_t *val){
  uint8_t buff[6];
  int32_t ret;

  ret = inv_iam20680_read_reg(s, MPUREG_ACCEL_XOUT_L, 6, buff);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

/* Data acquistion */
void IAM20680_ReadAccelerometer(float *pBuffer){
	/* Combine raw data as 16 bits signed integers */
	int16_t accRawSigned[3];
	IAM20680_acceleration_raw_get(&IMUDriver, accRawSigned);
	/* Convert to g and apply calibration equations */
	if(!isIMUAccCalibrated){
		pBuffer[0] = accRawSigned[0] * (0.122f * 0.001f);
		pBuffer[1] = accRawSigned[1] * (0.122f * 0.001f);
		pBuffer[2] = accRawSigned[2] * (0.122f * 0.001f);
	} else {
		pBuffer[0] = (((accRawSigned[0] * (0.122f * 0.001f)) * accSlope[0]) + accIntercept[0]);
		pBuffer[1] = (((accRawSigned[1] * (0.122f * 0.001f)) * accSlope[1]) + accIntercept[1]);
		pBuffer[2] = (((accRawSigned[2] * (0.122f * 0.001f)) * accSlope[2]) + accIntercept[2]);
//		/*convert to m/s^2 for testing with matlab*/
//		pBuffer[0] = (((accRawSigned[0] * (0.122f * 0.001f)) * accSlope[0]) + accIntercept[0])*9.81f;
//		pBuffer[1] = (((accRawSigned[1] * (0.122f * 0.001f)) * accSlope[1]) + accIntercept[1])*9.81f;
//		pBuffer[2] = (((accRawSigned[2] * (0.122f * 0.001f)) * accSlope[2]) + accIntercept[2])*9.81f;
	}
}

int32_t IAM20680_angular_rate_raw_get(struct inv_iam20680 * s , int16_t *val){
  uint8_t buff[6];
  int32_t ret;

  ret = inv_iam20680_read_reg(s, MPUREG_GYRO_XOUT_L, 6, buff);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

void IAM20680_ReadGyroscope(float *pBuffer){
	/* Combine raw data as 16 bits signed integers */
	int16_t gyroRawSigned[3];
	IAM20680_angular_rate_raw_get(&IMUDriver, gyroRawSigned);
	/* Convert data to rad/s and apply calibration offsets */
	if(!isIMUGyroCalibrated){
		pBuffer[0] = gyroRawSigned[0] * (8.75f * 0.001f);
		pBuffer[1] = gyroRawSigned[1] * (8.75f * 0.001f);
		pBuffer[2] = gyroRawSigned[2] * (8.75f * 0.001f);
	} else{
		pBuffer[0] = (gyroRawSigned[0] - gyroOffset[0]) * (8.75f * 0.001f) * (M_PI/180.0);
		pBuffer[1] = (gyroRawSigned[1] - gyroOffset[1]) * (8.75f * 0.001f) * (M_PI/180.0);
		pBuffer[2] = (gyroRawSigned[2] - gyroOffset[2]) * (8.75f * 0.001f) * (M_PI/180.0);
//		/*in deg/s if needed*/
//		pBuffer[0] = (gyroRawSigned[0] - gyroOffset[0]) * (8.75f * 0.001f);
//		pBuffer[1] = (gyroRawSigned[1] - gyroOffset[1]) * (8.75f * 0.001f);
//		pBuffer[2] = (gyroRawSigned[2] - gyroOffset[2]) * (8.75f * 0.001f);
	}

}


/* Device calibration */
//BOOL IAM20680_Calibrate(void){
//	uint8_t errorCounter = 0;
//
//	/* Blink for "REST_SECONDS" seconds to indicate user to leave the device on a flat surface for calibration */
//	uint8_t i = 0;
//	while(i != REST_SECONDS){
//		GPIO_WriteBit(GPIO_Pin_1, LED_ON);
//		SdkDelayMs(1000);
//		i++;
//	}
//	i = 0;
//
//	/*  Indicate that the device calibration will start */
//	GPIO_WriteBit(GPIO_Pin_1, LED_ON);
//	SdkDelayMs(250);
//
//	/* Activate data-ready interrupts on Pin INT1 */
//	//LSM6DSO32_dataReadyINT(ENABLE);
//
//	/* Wait for data-ready signal to stabilize */
//	SdkDelayMs(10);
//
//	/* Calibrate gyroscope */
//	IAM20680_CalibrateGyroscope();
//
//	/* Deactivate data-ready interrupts on pin INT1 */
//	//LSM6DSO32_dataReadyINT(DISABLE);
//
//	/* Wait for data-ready signal to stabilize */
//	SdkDelayMs(10);
//
//	/* Turn off LED */
//	GPIO_WriteBit(GPIO_Pin_1, LED_OFF);
//
//	/* Check if the device was successfully calibrated */
//	if(errorCounter != 0){
//		return FALSE;
//	} else{
//		return TRUE;
//	}
//}

//static void IAM20680_CalibrateGyroscope(void){
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
//			IAM20680_angular_rate_raw_get(&IMUDriver, rawDataBuffer);
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
