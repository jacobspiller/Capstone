

#ifndef USER_INC_INTERNALFLASH_H_
#define USER_INC_INTERNALFLASH_H_

#define		device_num_page 			0x2E
#define 	device_num_address 			0x1057600

#include <stdio.h>
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "BlueNRG1_adc.h"

BOOL IntFlash_ReadDeviceNum(uint8_t* deviceNum);
uint32_t IntFlash_ReadCali_1();
uint32_t IntFlash_ReadCali_2();
uint32_t IntFlash_ReadCali_3();
void IntFlash_WriteCali(uint16_t* cali);
void IntFlash_WriteDeviceNum(uint8_t* deviceNum);
void IntFlash_Test();


#endif /* USER_INC_INTERNALFLASH_H_ */
