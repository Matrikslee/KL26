#ifndef __ADC_H__
#define __ADC_H__

#include "sys.h"

//声明ADC端口地址
#define ADC0_DP0_PE20_DM0_PE21  (0xA860U)
#define ADC0_DP1_PE16_DM1_PE17  (0x2A060U)
#define ADC0_DP2_PE18_DM2_PE19  (0x4A460U)
#define ADC0_DP3_PE22_DM3_PE23  (0x6AC60U)
#define ADC0_SE1A_PE16					(0x26060U)
#define ADC0_SE5A_PE17					(0xA6260U)
#define ADC0_SE2A_PE18					(0x6460U)
#define ADC0_SE6A_PE19					(0xC6660U)
#define ADC0_SE0A_PE20					(0x6860U)
#define ADC0_SE4A_PE21					(0x86A60U)
#define ADC0_SE3A_PE22					(0x66C60U)
#define ADC0_SE7A_PE23					(0xE6E60U)

#define ADC0_SE4B_PE29					(0x87A60U)
#define ADC0_SE23A_PE30					(0x2E7C60U)
	 
#define ADC0_SE8A_PB0					  (0x104048U)
#define ADC0_SE9A_PB1					  (0x124248U)
#define ADC0_SE12A_PB2					(0x184448U)
#define ADC0_SE13A_PB3					(0x1A4648U)
#define ADC0_SE14A_PC0					(0x1C4050U)
#define ADC0_SE15A_PC1					(0x1E4250U)
#define ADC0_SE11A_PC2					(0x164450U)
#define ADC0_SE5B_PD1					  (0xA4258U)
#define ADC0_SE6B_PD5					  (0xC4A58U)
#define ADC0_SE7B_PD6					  (0xE4C58U)
	 
typedef struct
{
  uint32_t ADCxMap;              
	uint32_t ADC_Precision;
	uint16_t ADC_TriggerSelect; //触发源
}ADC_InitTypeDef;



// 精度
#define ADC_PRECISION_8BIT    (0x00U)
#define ADC_PRECISION_10BIT   (0x02U)
#define ADC_PRECISION_12BIT   (0x01U)
#define ADC_PRECISION_16BIT   (0x03U)


#define ADC_TRIGGER_HW     (uint16_t)(0)
#define ADC_TRIGGER_SW     (uint16_t)(1)



#define ADC_IT_AI       (uint16_t)(0)


#define ADC_DMAReq_COCO                     ((uint16_t)0)



#define A                 0x0
#define B                 0x1


void ADC_Init(ADC_InitTypeDef* ADC_InitStruct);
void ADC_DMACmd(ADC_Type* ADCx, uint16_t ADC_DMAReq, FunctionalState NewState);
uint32_t ADC_GetConversionValue(uint32_t ADCxMap);
void ADC_ITConfig(ADC_Type* ADCx,uint8_t ADC_Mux, uint16_t ADC_IT, FunctionalState NewState);
void ADC_ClearITPendingBit(ADC_Type* ADCx, uint8_t ADC_Mux, uint16_t ADC_IT);

#endif

