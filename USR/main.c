
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

#define duty_to_value(x) (uint16_t)(65535*x)

int main(void){
  //请认真确定你的外部晶振是否对应，8M请输入参数ClockSource_EX8M，
	//50M请输入参数ClockSource_EX50M。超频频率请使用 go to查看函数定义
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);

  DelayInit();
	ledInit(PTB,0);
	
  EnableInterrupts();
	
	PWMInit(PTA4,DIV1,65535);
	PWMInit(PTA12,DIV1,65535);	
	ADC_userInit();
	GPIO_userInit();
	
	while(1){
    DelayMs(500);
    twinkleLed(PTB,0);
	}
}




