#ifndef _CRC_H_
#define _CRC_H_

#include "uart.h"
#include "sys.h"

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OutPut_Data(void);
void sendData(void);
extern float OutData[4];

#endif

