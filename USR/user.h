
#ifndef __USR_H__
#define __USR_H__

#include "stdint.h"

//======================================================================
//获取ADC口信号函数
//入口：通道(channel):
//	0: 加速度计
//	1,2: 陀螺仪
//	3,4,5,6,7: 电磁传感器
//返回：信号值
//======================================================================
uint32_t ADC_GetValue(uint32_t channel);
//ADC初始化函数
void ADC_userInit(void);
//GPIO初始化函数
void GPIO_userInit(void);
//UART初始化函数
void UART_userInit(void);
#endif
