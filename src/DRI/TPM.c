#include "TPM.h"


//======================================================================
// 入口参数：Pin ：PWM输出引脚 
//         Div：时钟源分频数
//         modValue：模值，与初始化的modValue相关，Duty/modValue=实际占空比
//         且其大小决定了PWM输出的频率f=f(bus)/Div/modValue，最大为65535
// 返回参数：无
// 实现初始化PWM
//========================================================================
void PWMInit(uint8_t Pin, uint8_t Div, uint16_t modValue)
{    
	TPM_Type *pstTPMModule;
	uint8_t  TPMCh;
	//选择TPM时钟源
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
  // 配置PWM端口   
  switch (Pin)
    {
        case PTE24:
            
            PORTE->PCR[24] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH0;
            break;       
        
        case PTE25:
            
            PORTE->PCR[25] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH1;
            break;
            
        case PTE29:
            
            PORTE->PCR[29] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH2;
            break;
            
        case PTE30:
            
            PORTE->PCR[30] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK;
            pstTPMModule = TPM0; 
            TPMCh = CH3;
            break;  
                
        case PTE31:
            
            PORTE->PCR[31] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH4;
            break;       
        
        case PTA0:
            
            PORTA->PCR[0] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH5;
            break;
            
        case PTA3:
            
            PORTA->PCR[3] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH0;
            break;
            
        case PTA4:
            
            PORTA->PCR[4] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK;
            pstTPMModule = TPM0;
            TPMCh = CH1;
            break;
            
        case PTA5:
            
            PORTA->PCR[5] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH2;
            break;       
        
        case PTC8:
            
            PORTC->PCR[8] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK;
            pstTPMModule = TPM0; 
            TPMCh = CH4;
            break;
            
        case PTC9:
            
            PORTC->PCR[9] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM0;
            TPMCh = CH5;
            break;
            
        case PTA20:
            
            PORTA->PCR[20] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM1;
            TPMCh = CH0;
            break;
            
        case PTA21:
            
            PORTA->PCR[21] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM1;
            TPMCh = CH1;
            break;
            
        case PTA12:
            
            PORTA->PCR[12] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM1;
            TPMCh = CH0;
            break;
            
        case PTA13:
            
            PORTA->PCR[13] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM1;
            TPMCh = CH1;
            break;       
        
        case PTB0:
            
            PORTB->PCR[0] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM1;
            TPMCh = CH0;
            break;
            
        case PTB1:
            
            PORTB->PCR[1] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM1;
            TPMCh = CH1;
            break;
            
        case PTE22:
            
            PORTE->PCR[22] =PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;

        case PTE23:
            
            PORTE->PCR[23] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK;
            pstTPMModule = TPM2; 
            TPMCh = CH1;
            break;
            
        case PTA1:
            
            PORTA->PCR[1] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;
            
        case PTA2:
            
            PORTA->PCR[2] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;  
                
        case PTB2:
            
            PORTB->PCR[2] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;
            
        case PTB3:
            
            PORTB->PCR[3] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK;
            pstTPMModule = TPM2;
            TPMCh = CH1; 
            break;       
        
        case PTB18:
            
            PORTB->PCR[15] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK;
            pstTPMModule = TPM2; 
            TPMCh = CH0;
            break;
            
        case PTB19:
            
            PORTB->PCR[19] = PORT_PCR_MUX(0x3) | PORT_PCR_DSE_MASK; 
            pstTPMModule = TPM2;
            TPMCh = CH1;
            break;
     
        default:;
        
    }
    
// 2. 时钟使能:
    
  if (pstTPMModule == TPM0)   
    {
        SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
		}
    
  else if (pstTPMModule == TPM1)
    {
        SIM->SCGC6 |=SIM_SCGC6_TPM1_MASK;
			  
    }
    
  else if (pstTPMModule == TPM2)
    {
        SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
			  
    }
    
// 3. 配置 PWM:
//    
      Div &= 0x07; //取低三位，防止误输入大于7的分频数
			pstTPMModule->SC |= (TPM_SC_CMOD(1)             // CLKS=1:System clock(it's Bus clock here
		
											 |TPM_SC_PS(Div));   // f[ftm]=f[Bus]/(2^Div)
//		
			pstTPMModule->MOD   = modValue;           // 这里设置PWM频率，数值越大，占空比可调精度越高，
		                                            //但是PWM频率的高低跟电机效率，电机驱动性能，耗电量都相关，请仔细斟酌

			TPM_CnSC_REG(pstTPMModule,TPMCh) = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //先输出高后输出低           
}





//======================================================================
// 入口参数：Pin ：PWM输出引脚 
//           Duty：占空比，与初始化的modValue相关，Duty/modValue=实际占空比                                   
// 返回参数：无
// 实现更改PWM占空比
//========================================================================
void PWMOutput(uint8_t Pin, uint16_t Duty){
    TPM_Type *pstTPMModule;

    uint8_t  TPMCh;

    switch (Pin){
        case PTE24:
            pstTPMModule = TPM0;
            TPMCh = CH0;
            break;       
        
        case PTE25:
            pstTPMModule = TPM0;
            TPMCh = CH1;
            break;
            
        case PTE29:           
            pstTPMModule = TPM0;
            TPMCh = CH2;
            break;
            
        case PTE30:
            pstTPMModule = TPM0; 
            TPMCh = CH3;
            break;  
                
        case PTE31:
            pstTPMModule = TPM0;
            TPMCh = CH4;
            break;       
        
        case PTA0:
            pstTPMModule = TPM0;
            TPMCh = CH5;
            break;
            
        case PTA3: 
            pstTPMModule = TPM0;
            TPMCh = CH0;
            break;
            
        case PTA4:
            pstTPMModule = TPM0;
            TPMCh = CH1;
            break;
            
        case PTA5: 
            pstTPMModule = TPM0;
            TPMCh = CH2;
            break;       
        
        case PTC8:
            pstTPMModule = TPM0; 
            TPMCh = CH4;
            break;
            
        case PTC9:
            pstTPMModule = TPM0;
            TPMCh = CH5;
            break;
            
        case PTA20:
            pstTPMModule = TPM1;
            TPMCh = CH0;
            break;
            
        case PTA21:
            pstTPMModule = TPM1;
            TPMCh = CH1;
            break;
            
        case PTA12: 
            pstTPMModule = TPM1;
            TPMCh = CH0;
            break;
            
        case PTA13: 
            pstTPMModule = TPM1;
            TPMCh = CH1;
            break;       
        
        case PTB0:
            pstTPMModule = TPM1;
            TPMCh = CH0;
            break;
            
        case PTB1: 
            pstTPMModule = TPM1;
            TPMCh = CH1;
            break;
            
        case PTE22: 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;

        case PTE23:
            pstTPMModule = TPM2; 
            TPMCh = CH1;
            break;
            
        case PTA1: 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;
            
        case PTA2: 
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;  
                
        case PTB2:
            
            pstTPMModule = TPM2;
            TPMCh = CH0;
            break;
            
        case PTB3:
            pstTPMModule = TPM2;
            TPMCh = CH1; 
            break;       
        
        case PTB18:
            pstTPMModule = TPM2; 
            TPMCh = CH0;
            break;
            
        case PTB19:
            pstTPMModule = TPM2;
            TPMCh = CH1;
            break;

        default:break;
        
    }
   TPM_CnV_REG(pstTPMModule,TPMCh) = Duty;
}



