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

spdTypeDef spd;
angleTypeDef angle;
balanceDataTypeDef tmp_balance;
uint8_t cnt = 0;
const uint32_t pwmNumber = 4;
const uint8_t pwmArray[pwmNumber] = {PTA5, PTA12, PTE24, PTE25};

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

  DisableInterrupts();
	PWM_userInit(pwmArray, pwmNumber);                                                                                                                                                                                                                              	  
	ADC_userInit();
	GPIO_userInit();
	PIT_userInit();

	while(1){
		
		//getBalanceData(&tmp_balance);
		//kalmanFilter(&tmp_balance, &angle);
		int i;
		for ( i = 0; i < pwmNumber; ++i){
			PWMOutput(pwmArray[i],4000);
		}
		/*if(PIT_GetITStatus(PIT0, PIT_IT_TIF) == SET){
			PIT_ClearITPendingBit(PIT0, PIT_IT_TIF);
			switch(++cnt%5){
				case 1:
					getBalanceData(&tmp_balance);
					kalmanFilter(&tmp_balance, &angle);
					break;
				case 2:
					spd.m_spd_balance = (int32_t)limit(balanceControl(&tmp_balance, &angle),3000);
					break;
				case 0:
					motorControl(&spd);
					//twinkleLed(PTB,0);
					break;
				default:
					break;
			}
		}*/
	}
}
