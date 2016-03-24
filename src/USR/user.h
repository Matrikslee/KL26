
#ifndef __USR_H__
#define __USR_H__

#include "stdint.h"
#include "sys.h"

typedef struct{
  GPIO_Type* GPIO;
  uint16_t Pin;
} gpioPinTypeDef;

void PWM_userInit(const uint8_t* pwmArray, uint8_t len, uint32_t maxPwmDuty);

//ADC��ʼ������
void ADC_userInit(void);
uint32_t ADC_GetValue(uint8_t chl);
//======================================================================
//��ȡADC���źź���
//��ڣ�ͨ��(channel):
//	0,1,2: ���ٶȼƵ����¡����ҡ�ǰ��
//	3,4,5 : ������X��Z��Y
//	6,7,8,9: ��Ŵ�����
//���أ��ź�ֵ
//======================================================================

//GPIO��ʼ������
void GPIO_userInit(void);
//PIT��ʼ������
void PIT_userInit(void);
//DMA��ʼ������
void DMA_userInit(void);
#endif
