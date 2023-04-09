
#include "internalFlash.h"
extern int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
BOOL IntFlash_ReadDeviceNum(uint8_t* deviceNum){
	//Read from flash to get new device identifier
	deviceNum[0] = FLASH_ReadByte(device_num_address+3);
	deviceNum[1] = FLASH_ReadByte(device_num_address+2);
	deviceNum[2] = FLASH_ReadByte(device_num_address+1);
	if ( (0x30<=deviceNum[0]) && (deviceNum[0]<=0x39) && (0x30<=deviceNum[1]) && (deviceNum[1]<=0x39)
			&& (0x30<=deviceNum[2]) && (deviceNum[2]<=0x39) ){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

uint32_t IntFlash_ReadCali_1(){
	return FLASH_ReadWord(device_num_address + 4);
}

uint32_t IntFlash_ReadCali_2(){
	return FLASH_ReadWord(device_num_address + 8);
}

uint32_t IntFlash_ReadCali_3(){
	return FLASH_ReadWord(device_num_address + 12);
}


void IntFlash_WriteDeviceNum(uint8_t* deviceNum){
	uint8_t num2 = FLASH_ReadByte(device_num_address);
	uint8_t num1 = FLASH_ReadByte(device_num_address+1);
	uint8_t num0 = FLASH_ReadByte(device_num_address+2);
	//some info is changed, should erase and rewrite
	if ((num2 == 0xFF) && (num1 == 0xFF) && (num0 == 0xFF)){
		uint32_t identifier = 0;
		identifier += ('0'+deviceNum[2])<<8;
		identifier += ('0'+deviceNum[1])<<16;
		identifier += ('0'+deviceNum[0])<<24;
		FLASH_ProgramWord(device_num_address, identifier);
	}

	else if ((num0 != deviceNum[0]) || (num1 != deviceNum[1]) || (num2 != deviceNum[2])){
		//Back up
		uint32_t cali1 = IntFlash_ReadCali_1();
		uint32_t cali2 = IntFlash_ReadCali_2();
		uint32_t cali3 = IntFlash_ReadCali_3();

		//Erase
		FLASH_ErasePage(device_num_page);

		//Rewrite
		uint32_t identifier = 0;
		identifier += ('0'+deviceNum[2])<<8;
		identifier += ('0'+deviceNum[1])<<16;
		identifier += ('0'+deviceNum[0])<<24;
		FLASH_ProgramWord(device_num_address, identifier);
		FLASH_ProgramWord(device_num_address + 4, cali1);
		FLASH_ProgramWord(device_num_address + 8, cali2);
		FLASH_ProgramWord(device_num_address + 12, cali3);
	}
}

void IntFlash_WriteCali(uint16_t* cali)
{
	uint32_t cali1 = IntFlash_ReadCali_1();
	uint32_t cali2 = IntFlash_ReadCali_2();
	uint32_t cali3 = IntFlash_ReadCali_3();

	if ((cali1 == 0xFFFFFFFF) && (cali2 == 0xFFFFFFFF) && (cali3 == 0xFFFFFFFF)){
		FLASH_ProgramWord(device_num_address + 4, cali[0]);
		FLASH_ProgramWord(device_num_address + 8, cali[1]);
		FLASH_ProgramWord(device_num_address + 12, cali[2]);
	}
	//some info is changed, should erase and rewrite
	else if ((cali1 != cali[0]) || (cali2 != cali[1]) || (cali3 != cali[2])){
		//Back up
		uint32_t identifier = FLASH_ReadWord(device_num_address);
		//Erase
		FLASH_ErasePage(device_num_page);

		//Rewrite
		FLASH_ProgramWord(device_num_address, identifier);
		FLASH_ProgramWord(device_num_address + 4, cali[0]);
		FLASH_ProgramWord(device_num_address + 8, cali[1]);
		FLASH_ProgramWord(device_num_address + 12, cali[2]);
	}
}


void IntFlash_Test(){
	FLASH_ErasePage(device_num_page);

	uint8_t deviceNum[3];
	IntFlash_ReadDeviceNum(deviceNum);
	FLASH_ProgramWord(device_num_address + 4, 20);
	FLASH_ProgramWord(device_num_address + 8, 21);
	FLASH_ProgramWord(device_num_address + 12, 22);

	uint16_t a = IntFlash_ReadCali_1();
	uint16_t b = IntFlash_ReadCali_2();
	uint16_t c = IntFlash_ReadCali_3();

	SEGGER_RTT_printf(0,"device Num[0] is: %x, device Num[1] is: %x, device Num[2] is: %x, a is: %d, b is: %d, c is: %d\n",deviceNum[0],deviceNum[1],deviceNum[2],a,b,c);


	deviceNum[0] = 1;
	deviceNum[1] = 2;
	deviceNum[2] = 3;
	IntFlash_WriteDeviceNum(deviceNum);

	IntFlash_ReadDeviceNum(deviceNum);
	a = IntFlash_ReadCali_1();
	b = IntFlash_ReadCali_2();
	c = IntFlash_ReadCali_3();

	SEGGER_RTT_printf(0,"device Num[0] is: %x, device Num[1] is: %x, device Num[2] is: %x, a is: %d, b is: %d, c is: %d\n",deviceNum[0],deviceNum[1],deviceNum[2],a,b,c);
	uint16_t cal[3] = {200,3,55};
	IntFlash_WriteCali(cal);
	IntFlash_ReadDeviceNum(deviceNum);
	 a = IntFlash_ReadCali_1();
	 b = IntFlash_ReadCali_2();
	 c = IntFlash_ReadCali_3();

	 SEGGER_RTT_printf(0,"device Num[0] is: %x, device Num[1] is: %x, device Num[2] is: %x, a is: %d, b is: %d, c is: %d\n",deviceNum[0],deviceNum[1],deviceNum[2],a,b,c);

}

