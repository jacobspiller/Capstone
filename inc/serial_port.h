
#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

int return_xangle(void);
int return_yangle(void);
int return_zangle(void);

uint8_t SerialPort_DeviceInit(void);
void APP_Tick(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);

#endif // _SERIAL_PORT_H_
