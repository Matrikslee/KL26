
#ifndef __APP_H__
#define __APP_H__

#include "TPM.h"
#include "user.h"
#include "counter.h"
#include "stdint.h"
#include "tools.h"
#include <math.h>

//����ƽ�⻷ռ�ձ�
int32_t balanceCtrl(void);

//�����ٶȻ�ռ�ձ�
int32_t speedCtrl(void);

//���㷽��ռ�ձ�
int32_t directionCtrl(void);

//ʹ��ռ�ձȿ��Ƶ��
void motorControl(int32_t,int32_t,int32_t);

extern uint16_t time;

#endif
