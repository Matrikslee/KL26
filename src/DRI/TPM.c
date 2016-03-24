#include "TPM.h"


//======================================================================
// ��ڲ�����Pin ��PWM������� 
//         Div��ʱ��Դ��Ƶ��
//         modValue��ģֵ�����ʼ����modValue��أ�Duty/modValue=ʵ��ռ�ձ�
//         �����С������PWM�����Ƶ��f=f(bus)/Div/modValue�����Ϊ65535
// ���ز�������
// ʵ�ֳ�ʼ��PWM
//========================================================================
void PWMInit(uint8_t Pin, uint8_t Div, uint16_t modValue)
{    
	TPM_Type *pstTPMModule;
	uint8_t  TPMCh;
	//ѡ��TPMʱ��Դ
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
  // ����PWM�˿�   
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
    
// 2. ʱ��ʹ��:
    
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
    
// 3. ���� PWM:
//    
      Div &= 0x07; //ȡ����λ����ֹ���������7�ķ�Ƶ��
			pstTPMModule->SC |= (TPM_SC_CMOD(1)             // CLKS=1:System clock(it's Bus clock here
		
											 |TPM_SC_PS(Div));   // f[ftm]=f[Bus]/(2^Div)
//		
			pstTPMModule->MOD   = modValue;           // ��������PWMƵ�ʣ���ֵԽ��ռ�ձȿɵ�����Խ�ߣ�
		                                            //����PWMƵ�ʵĸߵ͸����Ч�ʣ�����������ܣ��ĵ�������أ�����ϸ����

			TPM_CnSC_REG(pstTPMModule,TPMCh) = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; //������ߺ������           
}





//======================================================================
// ��ڲ�����Pin ��PWM������� 
//           Duty��ռ�ձȣ����ʼ����modValue��أ�Duty/modValue=ʵ��ռ�ձ�                                   
// ���ز�������
// ʵ�ָ���PWMռ�ձ�
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



