#include "gpio.h"
#include "uart.h"
#include "delay.h"
#include "led.h"
#include "dma.h"
#include "spi.h"
#include "pit.h"
#include "adc.h"
#include "i2c.h"
#include "isr.h"
#include "accel.h"
#include "crc.h"
#include "pwm.h"
#include "stdio.h"
#include "TPM.h"
#include "user.h"
#include "app.h"
#include "counter.h"
#include "include.h"

dutyTypeDef output;
angleTypeDef angle;
balanceDataTypeDef tmp_balance;
directionDataTypeDef tmp_direction;
speedTypeDef speed;
const uint32_t pwmNumber = 4;
const uint8_t pwmArray[pwmNumber] = {PTA5, PTA12, PTE24, PTE25};
const uint32_t maxPwmDuty = 6000;
uint8_t cnt = 0;
int limit(int x, int lmt) {
	if(x>lmt) return lmt;
	if(x<-lmt) return -lmt;
	return x;
}

int main(void){
  //������ȷ������ⲿ�����Ƿ��Ӧ��8M���������ClockSource_EX8M��
	//50M���������ClockSource_EX50M����ƵƵ����ʹ�� go to�鿴��������
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);
	UART_PortInit(UART0_RX_PD06_TX_PD07,128000);
	UART_PortInit(UART1_RX_PE01_TX_PE00,96000);
  DelayInit();
	ledInit(PTB,0);
	ledInit(PTC,3);

  DisableInterrupts();
	PWM_userInit(pwmArray, pwmNumber, maxPwmDuty);                                                                                                                                                                                                                              	  
	ADC_userInit();
	GPIO_userInit();
	PIT_userInit();
	//��������ʼ��������������������������������
	Counter0_Init();
	Counter1_Init();
	
	while(1){		
		if(PIT_GetITStatus(PIT0, PIT_IT_TIF) == SET){
			PIT_ClearITPendingBit(PIT0, PIT_IT_TIF);
			cnt = 0;
			cnt++;
			if(cnt == 99) cnt = 0;
			output.leftDuty = output.rightDuty = 0;
			// balance handler
			getBalanceData(&tmp_balance);
			kalmanFilter(&tmp_balance, &angle);
			balanceCtrl(&angle, &output);
			// speed handler
			getSpeedData(&speed);
			speedCtrl(&speed,&output);
			// direction handler
			//getDirectionData(&tmp_direction);
			//directionCtrl(&tmp_direction, &output);
			
			//spd.m_spd_direction = (int32_t) limit(directionControl(), maxPwmDuty);
			motorControl(&output);
			}
		}
}
