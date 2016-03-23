#ifndef TIMER_H_
#define TIMER_H_
#include "ctype.h"
/*
** ###################################################################
**	Timer0�ж����ȼ��̶���Ϊ�������ȼ����
**	����Timer0_Init()�����Timer0_Enable()���ɿ�����ʱ�ж�
**	��ʱ�жϺ�����irq.c�ļ���
**	���³�ʼ����ʱ��������Ҫ��Timer0_Disable(),Ȼ��timer0_Init(val)���������ʱ������
**	����usΪ��ʱʱ�䣬��λΪ΢�룬sys_clkΪϵͳʱ��Ƶ��
** ###################################################################
*/
extern void Timer0_Init(uint32_t us,uint32_t sys_clk);			//us max value is 349525
extern void Timer0_Enable(void);
extern void Timer0_Disable(void);
/*
** ###################################################################
**	Timer1�ж����ȼ��������ã��β�prioΪָ�����ȼ���KL25ϵ�е�Ƭ������4�����ȼ�0-3��0���ȼ����
**	����Timer1_Init()�����Timer1_Enable()���ɿ�����ʱ�ж�
**	��ʱ�жϺ�����irq.c�ļ���
**	���³�ʼ����ʱ��������Ҫ��Timer1_Disable(),Ȼ��timer1_Init(val)���������ʱ������
**	����usΪ��ʱʱ�䣬��λΪ΢�룬sys_clkΪϵͳʱ��Ƶ�ʣ�prioΪ�ж����ȼ�����ԽС���ȼ�Խ��
** ###################################################################
*/
extern void Timer1_Init(uint32_t us,uint32_t prio,uint32_t sys_clk);		//us max value is 178956970,prio range is 0-3
extern void Timer1_Enable(void);
extern void Timer1_Disable(void);
#endif
