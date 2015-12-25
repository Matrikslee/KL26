#include "user.h"
#include "stdint.h"
#include "adc.h"
#include "gpio.h"
#include "uart.h"

const uint32_t adc_port[] = {ADC0_SE0A_PE20, ADC0_SE4A_PE21, ADC0_SE7A_PE23, ADC0_SE3A_PE22, ADC0_SE4B_PE29, ADC0_SE9A_PB1, ADC0_SE12A_PB2, ADC0_SE13A_PB3};

//======================================================================
//获取ADC口信号函数
//入口：通道(channel):
//	0: 加速度计
//	1,2: 陀螺仪
//	3,4,5,6,7: 电磁传感器
//返回：信号值
//======================================================================
uint32_t ADC_GetValue(uint32_t index){
	return ADC_GetConversionValue(adc_port[index]);
}

//ADC初始化函数
void ADC_userInit(void){
	ADC_InitTypeDef adc_initer;
	uint8_t i = 0;
	for (i = 0; i < 8; ++ i){
		adc_initer.ADCxMap = adc_port[i];
		adc_initer.ADC_Precision = ADC_PRECISION_16BIT;
		adc_initer.ADC_TriggerSelect = ADC_TRIGGER_SW;
		ADC_Init(&adc_initer);
	}
}

const uint16_t gpio_pin[] = {GPIO_Pin_10, GPIO_Pin_18, GPIO_Pin_3};

//GPIO初始化函数
void GPIO_userInit(void){
	GPIO_InitTypeDef gpio_initer;
	uint8_t i = 0;
	for (i = 0; i < 3; ++i){
		gpio_initer.GPIO_Pin = gpio_pin[i];
		gpio_initer.GPIO_InitState = Bit_RESET;
		gpio_initer.GPIO_IRQMode = GPIO_IT_RISING;
		gpio_initer.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio_initer.GPIOx = i<2?PTB:PTD;
		GPIO_Init(&gpio_initer);
	}
}

//UART初始化函数
void UART_userInit(void){
	UART_PortInit(UART1_RX_PE01_TX_PE00,128000); //参数二 波特率 待定
}
