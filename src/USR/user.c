#include "user.h"

#define GYRO_X_OFFSET (0x07FC)
#define GYRO_Y_OFFSET (0x0753)
#define GYRO_Z_OFFSET (0)
#define ACCZ_X_OFFSET (0)
#define ACCZ_Y_OFFSET (0x078E)
#define ACCZ_Z_OFFSET (0)
#define _PI (3.1415926f)

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

uint32_t ADC_CalxMap(uint8_t chl){
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

const uint8_t adc_number = 10;
const uint8_t adc_index[adc_number]={9,11,10,7,8,5,15,16,17,19};
//======================================================================
//获取ADC口信号函数
//入口：通道(channel):
//	0,1,2:   陀螺仪   X、Y、Z
//	3,4,5 :  加速度计 X、Y、Z
//	6,7,8,9: 电感
//返回：信号值
//======================================================================

uint32_t ADC_GetValue(uint8_t index){
	return ADC_GetConversionValue(ADC_CalxMap(adc_index[index]));
}

void GPIO_userInit(void){
	GPIO_InitTypeDef initer;
	initer.GPIO_InitState = Bit_RESET;
	initer.GPIO_IRQMode = GPIO_IT_DISABLE;
	initer.GPIO_Mode = GPIO_Mode_IPU;
	
	initer.GPIOx = PTB;
	initer.GPIO_Pin = 9;
	GPIO_Init(&initer);
	
	initer.GPIO_Pin = 10;
	GPIO_Init(&initer);
	
	initer.GPIOx = PTE;
	initer.GPIO_Pin = 3;
	GPIO_Init(&initer);
}

void PIT_userInit(void){
	PIT_InitTypeDef initer;
	initer.PITx = PIT0;
	initer.PIT_Interval = 5; //单位MS
	PIT_Init(&initer);
}

void PWM_userInit(){
	PWMInit(PWM_LEFT,  DIV1, MAX_PWM_DUTY);
	PWMInit(PWM_RIGHT, DIV1, MAX_PWM_DUTY);
}

static const uint8_t imu_number = 6;
static float imu_ratio[imu_number] = {0.23578,0.120248,0,0,0.086,0};
static uint32_t imu_offset[imu_number] = {GYRO_X_OFFSET,GYRO_Y_OFFSET,GYRO_Z_OFFSET,ACCZ_X_OFFSET, ACCZ_Y_OFFSET, ACCZ_Z_OFFSET};

//陀螺仪零偏值初始化函数
void gyro_offsetInit(void) {
	const uint8_t smaple_amount = 10;
	const uint8_t gyro_number = 3;
	int i, j;
	for ( i = 0; i < gyro_number; ++ i) {
		uint32_t sample = 0, max_t = 0, min_t = INT32_MAX;
		for ( j = 0; j < smaple_amount; ++ j) {
			uint32_t tmp = ADC_GetValue(i)>>4;
			max_t = tmp > max_t?tmp:max_t;
			min_t = tmp < min_t?tmp:min_t;
			sample += tmp;
		}
		imu_offset[i] = (sample-min_t-max_t) / (smaple_amount-2);
	}
}

void IMU_userInit(){
	ADC_InitTypeDef initer;
	initer.ADC_Precision = ADC_PRECISION_16BIT;
	initer.ADC_TriggerSelect = ADC_TRIGGER_SW;
	
	initer.ADCxMap = ADC_CalxMap(adc_index[ACCZ_Y]);
	ADC_Init(&initer);
	
	initer.ADCxMap = ADC_CalxMap(adc_index[GYRO_X]);
	ADC_Init(&initer);
	
	initer.ADCxMap = ADC_CalxMap(adc_index[GYRO_Y]);
	ADC_Init(&initer);
}

void inductance_userInit(void){
	ADC_InitTypeDef initer;
	initer.ADC_Precision = ADC_PRECISION_16BIT;
	initer.ADC_TriggerSelect = ADC_TRIGGER_SW;
	initer.ADCxMap = ADC_CalxMap(adc_index[INDUCTANCE_LL]);
	ADC_Init(&initer);
	initer.ADCxMap = ADC_CalxMap(adc_index[INDUCTANCE_LR]);
	ADC_Init(&initer);
	initer.ADCxMap = ADC_CalxMap(adc_index[INDUCTANCE_RL]);
	ADC_Init(&initer);
	initer.ADCxMap = ADC_CalxMap(adc_index[INDUCTANCE_RR]);
	ADC_Init(&initer);
}


float getIMUValue(uint8_t index){
	int32_t tmp_value;
	tmp_value = ADC_GetValue(index)>>4;
	return ((float)tmp_value-imu_offset[index])*imu_ratio[index];
}

float getAcczValue(uint8_t index) {
	float tmp = getIMUValue(index);
	return asin((flimit(tmp,100))/100.)*180/_PI;
}

float getGyroValue(uint8_t index){
	return getIMUValue(index);
}

float getInductanceValue(uint8_t index){
	return ADC_GetValue(index)>>8;
}
