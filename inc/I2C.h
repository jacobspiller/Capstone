
#ifndef USER_INC_I2C_H_
#define USER_INC_I2C_H_

#include "bluenrg_x_device.h"
#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

void I2C_test(void);

void I2C_ConfigurationMaster(void);
//I2C_AppOpStatus I2C_Read_Reg(uint8_t dev_address, uint8_t send_val, uint8_t* get_val);
void processCommand(void);
//void helpMessage(void);

#endif /* USER_INC_I2C_H_ */
