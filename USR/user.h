
#ifndef __USR_H__
#define __USR_H__

#include "stdint.h"
#include "sys.h"

typedef struct{
  GPIO_Type* GPIO;
  uint16_t Pin;
} gpioPinTypeDef;

extern uint8_t pwmLeft;
extern uint8_t pwmRight;

//ADC初始化函数
void ADC_userInit(void);
//======================================================================
//获取ADC口信号函数
//入口：通道(channel):
//	0: 加速度计
//	1,2: 陀螺仪
//	3,4,5,6,7: 电磁传感器
//返回：信号值
//======================================================================
uint32_t ADC_GetValue(uint8_t chl);
//GPIO初始化函数
void GPIO_userInit(void);
//PIT初始化函数
void PIT_userInit(void);
#endif
