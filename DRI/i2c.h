
#ifndef __I2C_H__
#define __I2C_H__
 
#include "sys.h"

typedef struct
{
	uint32_t I2CxMAP;
  uint32_t I2C_ClockSpeed;        
}I2C_InitTypeDef;													


//I2C 速度
#define I2C_CLOCK_SPEED_50KHZ                   ( 50*1000)
#define I2C_CLOCK_SPEED_100KHZ                  (100*1000)
#define I2C_CLOCK_SPEED_150KHZ                  (150*1000)
#define I2C_CLOCK_SPEED_200KHZ                  (200*1000)
#define I2C_CLOCK_SPEED_250KHZ                  (250*1000)
#define I2C_CLOCK_SPEED_300KHZ                  (300*1000)


//引脚
#define I2C0_SCL_PE01_SDA_PE00  (0x81A0U)
#define I2C0_SCL_PE19_SDA_PE18  (0xA520U)
#define I2C0_SCL_PE24_SDA_PE25  (0xB160U)
#define I2C1_SCL_PA03_SDA_PA04  (0x8681U)
#define I2C0_SCL_PB00_SDA_PB01  (0x8088U)
#define I2C0_SCL_PB02_SDA_PB03  (0x8488U)
#define I2C1_SCL_PC01_SDA_PC02  (0x8291U)
#define I2C0_SCL_PC08_SDA_PC09  (0x9090U)
#define I2C1_SCL_PC10_SDA_PC11  (0x9491U)

//中断配置
#define I2C_IT_TCF             (uint16_t)(0)
#define I2C_IT_IAAS            (uint16_t)(1)
#define I2C_IT_ARBL            (uint16_t)(2)
#define I2C_IT_SLTF            (uint16_t)(3)
#define I2C_IT_SHTF2           (uint16_t)(4)


#define I2C_DMAReq_TCF             ((uint16_t)0)


//I2C读写方向
#define I2C_MASTER_WRITE   (0)
#define I2C_MASTER_READ    (1)

//接口函数
void I2C_Init(I2C_InitTypeDef* I2C_InitStruct);
void I2C_GenerateSTART(I2C_Type *I2Cx);
void I2C_GenerateRESTART(I2C_Type *I2Cx);
void I2C_GenerateSTOP(I2C_Type *I2Cx);
void I2C_SendData(I2C_Type *I2Cx,uint8_t data8);
void I2C_Send7bitAddress(I2C_Type* I2Cx, uint8_t Address, uint8_t I2C_Direction);
uint8_t I2C_WaitAck(I2C_Type *I2Cx);
void I2C_SetMasterMode(I2C_Type* I2Cx,uint8_t I2C_Direction);
void I2C_GenerateAck(I2C_Type *I2Cx);
void I2C_GenerateNAck(I2C_Type *I2Cx);
void I2C_EnableAck(I2C_Type *I2Cx);
void I2C_ITConfig(I2C_Type* I2Cx, uint16_t I2C_IT, FunctionalState NewState);
ITStatus I2C_GetITStatus(I2C_Type* I2Cx, uint16_t I2C_IT);
void I2C_DMACmd(I2C_Type* I2Cx, uint16_t I2C_DMAReq, FunctionalState NewState);
void I2C_ClearITPendingBit(I2C_Type* I2Cx, uint16_t I2C_IT);
uint8_t I2C_ReadData(I2C_Type *I2Cx);
uint8_t I2C_IsLineBusy(I2C_Type* I2Cx);


#endif


