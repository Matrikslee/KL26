#include "user.h"
#include "stdint.h"
#include "adc.h"
#include "gpio.h"
#include "uart.h"
#include "pit.h"
#include "sys.h"
#include "TPM.h"
#include "counter.h"
#include "dma.h"

static const PeripheralMapTypeDef ADC_Check_Maps[] =
{
	{0, 4, 1,20, 2, 0, 0},  //ADC0_DP0_PE20_DM0_PE21 0
	{0, 4, 1,16, 2, 1, 0},  //ADC0_DP1_PE16_DM1_PE17 1
	{0, 4, 1,18, 2, 2, 0},  //ADC0_DP2_PE18_DM2_PE19 2
	{0, 4, 1,22, 2, 3, 0},  //ADC0_DP3_PE22_DM3_PE23 3
	{0, 4, 1,16, 1, 1, 0},  //ADC0_SE1A_PE16 4
	{0, 4, 1,17, 1, 5, 0},  //ADC0_SE5A_PE17 5
	{0, 4, 1,18, 1, 0, 0},  //ADC0_SE2A_PE18 6
	{0, 4, 1,19, 1, 6, 0},  //ADC0_SE6A_PE19 7
	{0, 4, 1,20, 1, 0, 0},  //ADC0_SE0A_PE20 8
	{0, 4, 1,21, 1, 4, 0},  //ADC0_SE4A_PE21 9
	{0, 4, 1,22, 1, 3, 0},  //ADC0_SE3A_PE22 10
	{0, 4, 1,23, 1, 7, 0},  //ADC0_SE7A_PE23 11
	{0, 4, 1,29, 1, 4, 1},  //ADC0_SE4B_PE29 12
	{0, 4, 1,30, 1,23, 0},  //ADC0_SE23A_PE30 13
	{0, 1, 1, 0, 1, 8, 0},  //ADC0_SE8A_PB0 14
	{0, 1, 1, 1, 1, 9, 0},  //ADC0_SE9A_PB1 15
	{0, 1, 1, 2, 1,12, 0},  //ADC0_SE12A_PB2 16
	{0, 1, 1, 3, 1,13, 0},  //ADC0_SE13A_PB3 17
	{0, 2, 1, 0, 1,14, 0},  //ADC0_SE14A_PC0 18
	{0, 2, 1, 1, 1,15, 0},  //ADC0_SE15A_PC1 19
	{0, 2, 1, 2, 1,11, 0},  //ADC0_SE11A_PC2 20
	{0, 3, 1, 1, 1, 5, 1},  //ADC0_SE5B_PD1 21
	{0, 3, 1, 5, 1, 6, 1},  //ADC0_SE6B_PD5 22
	{0, 3, 1, 6, 1, 7, 1},  //ADC0_SE7B_PD6 23
};

const uint8_t adc_channel_length = 10;
const uint8_t adc_channel_index[adc_channel_length]={5,7,8,9,10,11,15,16,17,19};

uint32_t ADC_CalxMap(uint8_t chl)
{
	uint32_t value = 0;
	value =   ADC_Check_Maps[chl].m_ModuleIndex<<0;
	value |=  ADC_Check_Maps[chl].m_PortIndex <<3;
	value |=  ADC_Check_Maps[chl].m_MuxIndex<<6;
	value |=  ADC_Check_Maps[chl].m_PinBaseIndex<<9;
	value |=  ADC_Check_Maps[chl].m_PinCntIndex<<14;
	value |=  ADC_Check_Maps[chl].m_ChlIndex<<17;
	value |=  ADC_Check_Maps[chl].m_SpecDefine1<<22;
	return value;
}

uint32_t ADC_GetValue(uint8_t index){
	return ADC_GetConversionValue(ADC_CalxMap(adc_channel_index[index]));
}

//ADC初始化函数
void ADC_userInit(void){
	ADC_InitTypeDef adc_initer;
	uint8_t i = 0;
	for (i = 0; i < adc_channel_length; ++ i){
		adc_initer.ADCxMap = ADC_CalxMap(adc_channel_index[i]);
		adc_initer.ADC_Precision = ADC_PRECISION_16BIT;
		adc_initer.ADC_TriggerSelect = ADC_TRIGGER_SW;
		ADC_Init(&adc_initer);
	}
}

const uint32_t gpio_pin_length = 3;
const gpioPinTypeDef gpio_pin[gpio_pin_length] = {{PTB, 9}, {PTB,10}, {PTE,3}};

//GPIO初始化函数
void GPIO_userInit(void){
	GPIO_InitTypeDef gpio_initer;
	uint8_t i = 0;
	for (i = 0; i < gpio_pin_length; ++i){
		gpio_initer.GPIO_Pin = gpio_pin[i].Pin;
		gpio_initer.GPIO_InitState = Bit_RESET;
		gpio_initer.GPIO_IRQMode = GPIO_IT_DISABLE;
		gpio_initer.GPIO_Mode = GPIO_Mode_IPU;
		gpio_initer.GPIOx = gpio_pin[i].GPIO;
		GPIO_Init(&gpio_initer);
	}
}

//PIT初始化函数
void PIT_userInit(void){
	PIT_InitTypeDef pit_initer;
	pit_initer.PITx = PIT0;
	pit_initer.PIT_Interval = 1; //单位MS
	PIT_Init(&pit_initer);
}

//DMA初始化函数
void DMA_userInit(void){
	DMA_InitTypeDef dma_initer;
	dma_initer.Channelx = 0;
	DMA_Init(&dma_initer);
	
}

void PWM_userInit(const uint8_t* pwmArray, uint8_t len, uint32_t maxPwmDuty){
	int i;
	for ( i = 0; i < len; ++i){
		PWMInit(pwmArray[i],DIV1,maxPwmDuty);
	}
}
