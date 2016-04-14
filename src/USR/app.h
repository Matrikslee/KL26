
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"

//计算平衡环占空比
int32_t balanceCtrl(void);

//计算速度环占空比
int32_t speedCtrl(void);

//计算方向环占空比
int32_t directionCtrl(void);

//使用占空比控制电机
void motorControl(int32_t,int32_t,int32_t);

#endif
