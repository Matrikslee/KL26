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
#include "include.h"

spdTypeDef spd;
angleTypeDef angle;
balanceDataTypeDef tmp_balance;
uint8_t cnt = 0;
const uint32_t pwmNumber = 4;
const uint8_t pwmArray[pwmNumber] = {PTA5, PTA12, PTE24, PTE25};
const uint32_t maxPwmDuty = 6000;

int limit(int x, int lmt) {
	if(x>lmt) return lmt;
	if(x<-lmt) return -lmt;
	return x;
}

int main(void){
  //请认真确定你的外部晶振是否对应，8M请输入参数ClockSource_EX8M，
	//50M请输入参数ClockSource_EX50M。超频频率请使用 go to查看函数定义
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

	while(1){
		
		//getBalanceData(&tmp_balance);
		
		if(PIT_GetITStatus(PIT0, PIT_IT_TIF) == SET){
			PIT_ClearITPendingBit(PIT0, PIT_IT_TIF);
//			switch(++cnt%5){
//				case 0:
					getBalanceData(&tmp_balance);
					kalmanFilter(&tmp_balance, &angle);
//					break;
//				case 1:
					spd.m_spd_balance = (int32_t)limit(balanceControl(&angle),maxPwmDuty);
//					break;
//				case 4:
					motorControl(&spd);
//					break;
//				default:
//					break;
			}
		}
//	}
}
