
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

int main(void){
  //请认真确定你的外部晶振是否对应，8M请输入参数ClockSource_EX8M，
	//50M请输入参数ClockSource_EX50M。超频频率请使用 go to查看函数定义
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);

  DelayInit();
	ledInit(PTB,0);
	
  DisableInterrupts();
	
	PWMInit(PTA4,DIV1,65535);
	PWMInit(PTA12,DIV1,65535);
	//ADC_userInit();
	//GPIO_userInit();
	//PIT_userInit();
	
	while(1){
		/*
		balanceDataTypeDef tmp_balance;
		spdTypeDef spd;
		uint8_t Tim = timer();
		switch(Tim){
			case 1:
				getBalanceData(&tmp_balance);
				break;
			case 2:
				spd.m_spd_balance = balanceControl(&tmp_balance);
				break;
			case 5:
				motorControl(&spd);
				break;
			default:
				break;
		}*/
		PWMOutput(PTA4, 0x8000);
		PWMOutput(PTA12, 0x7000);
	}
}
