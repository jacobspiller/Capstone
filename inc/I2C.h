/*
 * I2C.h
 *
 *  Created on: Jun 13, 2022
 *      Author: Mona
 */

#ifndef USER_INC_I2C_H_
#define USER_INC_I2C_H_

#include "bluenrg_x_device.h"
#include <stdio.h>
//#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#define CS_IMU				GPIO_Pin_5 = 1 //Set the CS for the IMU to high, is this the right place to do it?
#define INT1_IMU			GPIO_Pin_4 = 1 //have to create ints, where and how?

#define whoAmIDat 0x6C

void I2C_test(void);

void I2C_ConfigurationMaster(void);
//I2C_AppOpStatus I2C_Read_Reg(uint8_t dev_address, uint8_t send_val, uint8_t* get_val);
void processCommand(void);
//void helpMessage(void);

#endif /* USER_INC_I2C_H_ */
