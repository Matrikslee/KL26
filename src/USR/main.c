#include "stdio.h"
#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "adc.h"
#include "TPM.h"
#include "counter.h"
#include "user.h"
#include "app.h"

#define START_DELAY (200)
#define DELAY_DELTA (100)

int32_t balance = 0, speed = 0, turn = 0;
uint16_t time = 0;

int main(void){
  //请认真确定你的外部晶振是否对应，8M请输入参数ClockSource_EX8M，
	//50M请输入参数ClockSource_EX50M。超频频率请使用 go to查看函数定义
  SystemClockSetup(ClockSource_EX50M,CoreClock_120M);
	UART_PortInit(UART0_RX_PD06_TX_PD07,128000);
	
	PWM_userInit();

	PWMOutput(PWM_LEFT, HALF_MAX_PWM_DUTY);
	PWMOutput(PWM_RIGHT,HALF_MAX_PWM_DUTY);
	
	GPIO_userInit();
	PIT_userInit();
	IMU_userInit();
	inductance_userInit();
	Counter0_Init();
	Counter1_Init();
	
	while(1){
		
		if(PIT_GetITStatus(PIT0, PIT_IT_TIF) == SET){
			PIT_ClearITPendingBit(PIT0, PIT_IT_TIF);
			
			if(time == START_DELAY-DELAY_DELTA) { gyro_offsetInit(); }
			if(time < 3000) { ++time; }
			
			balance = balanceCtrl();
			speed = speedCtrl();
			turn = directionCtrl();
		}
		
		if(time < START_DELAY) {
			balance = speed = turn = 0;
		}
		if(time < START_DELAY+DELAY_DELTA) {
			speed = turn = 0;
		}
		
		motorControl(balance, speed, turn);
	}
}
