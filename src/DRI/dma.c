#include "dma.h"



void DMA_Init(DMA_InitTypeDef *DMA_InitStruct)
{
  // 开启时钟门 
  SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;    
  SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
  //   配置触发源     
  if(DMA_InitStruct->PeripheralDMAReq == DMA_SOFTWARE_TRIGGER)
  {
    // 禁止外围请求
    DMA0->DMA[DMA_InitStruct->Channelx].DCR &= ~DMA_DCR_ERQ_MASK;
  }
  else
  {
    // 使能外围请求
    DMAMUX0->CHCFG[DMA_InitStruct->Channelx] = DMAMUX_CHCFG_SOURCE(DMA_InitStruct->PeripheralDMAReq);
    //  配置周期触发
    (DMA_InitStruct->PeriodicModeState == ENABLE)?(DMAMUX0->CHCFG[DMA_InitStruct->Channelx]|= DMAMUX_CHCFG_TRIG_MASK):(DMAMUX0->CHCFG[DMA_InitStruct->Channelx]&= ~DMAMUX_CHCFG_TRIG_MASK);
  }
  //   配置DMA地址 
  DMA0->DMA[DMA_InitStruct->Channelx].SAR  = DMA_InitStruct->SourceBaseAddr;
  //  配置dma目标地址 
  DMA0->DMA[DMA_InitStruct->Channelx].DAR  = DMA_InitStruct->DestBaseAddr;
    
  DMA0->DMA[DMA_InitStruct->Channelx].DCR &= (DMA_DCR_SSIZE_MASK | DMA_DCR_DSIZE_MASK);
  DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_SSIZE(DMA_InitStruct->SourceDataSize);
  DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_DSIZE(DMA_InitStruct->DestDataSize);
 
  (DMA_InitStruct->SourceInc == ENABLE)?(DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_SINC_MASK):(DMA0->DMA[DMA_InitStruct->Channelx].DCR &= ~DMA_DCR_SINC_MASK);
  (DMA_InitStruct->DestInc == ENABLE)?(DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_DINC_MASK):(DMA0->DMA[DMA_InitStruct->Channelx].DCR &= ~DMA_DCR_DINC_MASK);
  //  配置转移字节            
  DMA0->DMA[DMA_InitStruct->Channelx].DSR_BCR &= ~DMA_DSR_BCR_BCR_MASK;
  DMA0->DMA[DMA_InitStruct->Channelx].DSR_BCR |=  DMA_DSR_BCR_BCR(DMA_InitStruct->BytesToTransfer);
  //  CS 配置 若CS = 0  DMA 继续转移,所以一般情况下cs = 1
  (DMA_InitStruct->CycleStealState == ENABLE)?(DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_CS_MASK):(DMA0->DMA[DMA_InitStruct->Channelx].DCR &= ~DMA_DCR_CS_MASK);
  //  立即转移配置
  if(DMA_InitStruct->PeripheralDMAReq == DMA_SOFTWARE_TRIGGER)
  {
    DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_START_MASK;
  }
  else
  {
    DMA0->DMA[DMA_InitStruct->Channelx].DCR |= DMA_DCR_ERQ_MASK;
  }
  // 使能通道1   
  DMAMUX0->CHCFG[DMA_InitStruct->Channelx] |= DMAMUX_CHCFG_ENBL_MASK;
}


uint8_t DMA_IsComplete(uint8_t DMAChl)
{
  if(DMA0->DMA[DMAChl].DSR_BCR & DMA_DSR_BCR_DONE_MASK)
  {
 
    DMA0->DMA[DMAChl].DSR_BCR |= DMA_DSR_BCR_DONE_MASK;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

//DMA IT 配置
void DMA_ITConfig(uint16_t DMA_IT, uint8_t DMA_CH, FunctionalState NewState)
{
  switch(DMA_IT)
  {
    case DMA_IT_MAJOR:
    (ENABLE == NewState)?(DMA0->DMA[DMA_CH].DCR |= DMA_DCR_EINT_MASK):( DMA0->DMA[DMA_CH].DCR &= ~DMA_DCR_EINT_MASK);
    break;
    default:break;
  }
}

//DMA 清楚中断标志位
void DMA_ClearITPendingBit(uint16_t DMA_IT, uint8_t DMA_CH)
{
  DMA0->DMA[DMA_CH].DSR_BCR |= DMA_DSR_BCR_DONE_MASK;  
}
