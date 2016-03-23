#include "MKL25Z4.H"
#include "ctype.h"

void Timer0_Init(uint32_t us,uint32_t sys_clk)
{
	uint32_t max_val=0;
	max_val=us*(sys_clk/100000);//100000
	//if(max_val>=0x00FFFFFFUL)
	//	max_val=0x00FFFFFFUL;
	SysTick->CTRL &=~(uint32_t)(0x1UL);
	SysTick->CTRL |=(uint32_t)(0x1UL<<1);
	SysTick->CTRL |=(uint32_t)(0x1UL<<2);
	SysTick->LOAD=max_val;
	//SysTick->CTRL |=(uint32_t)(0x1UL);
}
void Timer0_Enable(void)
{
	SCB->ICSR	|=(uint32_t)(0x1UL<<25);
	SysTick->CTRL |=(uint32_t)(0x1UL);
}
void Timer0_Disable(void)
{
	SysTick->CTRL &=~(uint32_t)(0x1UL);
}
extern void Timer0_IRQ(void);
void SysTick_Handler(void)
{
	Timer0_IRQ();
}

#define PIT_IRQ_NUM	22
void Timer1_Init(uint32_t us,uint32_t prio,uint32_t sys_clk)
{
	/*Configure the IRQ of the PIT*/
	NVIC->ICPR[((uint32_t)(PIT_IRQ_NUM) >> 5)] = (1 << ((uint32_t)(PIT_IRQ_NUM) & 0x1F));
	NVIC->IP[((uint32_t)(PIT_IRQ_NUM) >> 2)]=prio<<6;
	NVIC->ISER[((uint32_t)(PIT_IRQ_NUM) >> 5)] = (1 << ((uint32_t)(PIT_IRQ_NUM) & 0x1F)); /* enable interrupt */
	
	SIM->SCGC6 |=(uint32_t)(0x1UL<<23);			//open the clock of the PIT
	
	PIT->MCR	=0;
	PIT->CHANNEL[0].LDVAL=us*(sys_clk/2000000);//2000000
}

void Timer1_Enable(void)
{
		PIT->CHANNEL[0].TFLG =0x1UL;
		PIT->CHANNEL[0].TCTRL |=0x3UL;
}
void Timer1_Disable(void)
{
	PIT->CHANNEL[0].TCTRL =0;
}
extern void Timer1_IRQ(void);
void PIT_IRQHandler(void)
{
	PIT->CHANNEL[0].TFLG =0x1UL;
	Timer1_IRQ();
}

