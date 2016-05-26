#include "MKL25Z4.h"
#include "ctype.h"

void Counter0_Init(void)
{
	/*First configure the pin of the module will be use*/
	SIM->SCGC5 |=(uint32_t)(0x1UL<<11);			//open the clock of the PTC
	PORTC->PCR[5] &=~(uint32_t)(0x7UL<<8);
	PORTC->PCR[5] |=(uint32_t)(0x3UL<<8);	//PTC12 for ALT3(LPTMR_ALT2)

	/*Configure the module*/
	SIM->SCGC5 |=(uint32_t)(0x1UL<<0);			//open the clock of the LPTMR0
	LPTMR0->CSR =0;
	LPTMR0->PSR	=0;
  LPTMR0->CMR	=0xFFFFUL;
	LPTMR0->PSR	|=(uint32_t)(0x1UL<<2);		//prescaler and glitch filter bypassed,if you want to use it,reference the book
	
	LPTMR0->CSR	&=~(uint32_t)(0x3UL<<4);
	LPTMR0->CSR |=(uint32_t)(0x2UL<<4);		//Select the LPTMR_ALT2 as the input clk
	LPTMR0->CSR |=(uint32_t)(0x1UL<<1);		//counter mode selected
	
	LPTMR0->CSR	|=(uint32_t)0x1UL;			//enable the counter
}
uint32_t Counter0_Read(void)
{
	uint32_t temp=0;
	LPTMR0->CNR =0x00;
	temp=LPTMR0->CNR;
	return temp;
}
void Counter0_Clear(void)
{
	LPTMR0->CSR =0;
	LPTMR0->PSR	=0;
  LPTMR0->CMR	=0xFFFFUL;
	LPTMR0->PSR	|=(uint32_t)(0x1UL<<2);		//prescaler and glitch filter bypassed,if you want to use it,reference the book
	
	LPTMR0->CSR	&=~(uint32_t)(0x3UL<<4);
	LPTMR0->CSR |=(uint32_t)(0x2UL<<4);		//Select the LPTMR_ALT2 as the input clk
	LPTMR0->CSR |=(uint32_t)(0x1UL<<1);		//counter mode selected
	
	LPTMR0->CSR	|=(uint32_t)0x1UL;			//enable the counter
}

void Counter1_Init(void)
{
	SIM->SCGC5 |=(uint32_t)(0x1UL<<11);		//open the clock of the PTC
	PORTC->PCR[13] &=~(uint32_t)(0x7UL<<8);
	PORTC->PCR[13] |=(uint32_t)(0x4UL<<8);		//PTC13 ALT4 for TPM_CLKIN1
	
	SIM->SOPT4 |=(uint32_t)(0x1UL<<26);				//select the TPM_CLKIN1 as the counter resource
	
	SIM->SCGC6 |=(uint32_t)(0x1UL<<26);			//open the clock of the TPM2
	
	TPM2->CONF |=(uint32_t)(0x3UL<<6);				//contine in debug mode
	TPM2->CNT=0;
	TPM2->MOD =(uint32_t)0xFFFFUL;	
	TPM2->SC =(uint32_t)(0x2UL<<3);					//select the external clk
}
uint32_t Counter1_Read(void)
{
	uint32_t temp=0;
	temp=TPM2->CNT;
	return temp;
}

void Counter1_Clear(void)
{
	TPM2->CNT=0;
}
