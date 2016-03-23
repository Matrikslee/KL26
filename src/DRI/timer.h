#ifndef TIMER_H_
#define TIMER_H_
#include "ctype.h"
/*
** ###################################################################
**	Timer0中断优先级固定，为所有优先级最大
**	调用Timer0_Init()后调用Timer0_Enable()即可开启定时中断
**	定时中断函数在irq.c文件中
**	重新初始化定时器周期需要先Timer0_Disable(),然后timer0_Init(val)，最后开启定时器即可
**	参数us为定时时间，单位为微秒，sys_clk为系统时钟频率
** ###################################################################
*/
extern void Timer0_Init(uint32_t us,uint32_t sys_clk);			//us max value is 349525
extern void Timer0_Enable(void);
extern void Timer0_Disable(void);
/*
** ###################################################################
**	Timer1中断优先级可以配置，形参prio为指定优先级，KL25系列单片机具有4个优先级0-3，0优先级最高
**	调用Timer1_Init()后调用Timer1_Enable()即可开启定时中断
**	定时中断函数在irq.c文件中
**	重新初始化定时器周期需要先Timer1_Disable(),然后timer1_Init(val)，最后开启定时器即可
**	参数us为定时时间，单位为微秒，sys_clk为系统时钟频率，prio为中断优先级，数越小优先级越高
** ###################################################################
*/
extern void Timer1_Init(uint32_t us,uint32_t prio,uint32_t sys_clk);		//us max value is 178956970,prio range is 0-3
extern void Timer1_Enable(void);
extern void Timer1_Disable(void);
#endif
