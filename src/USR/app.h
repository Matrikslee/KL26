
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"

typedef struct {
	float m_angle;
	float m_rate;
} angleTypeDef;

typedef struct {
	int32_t m_spd_speed;
	int32_t m_spd_balance;
	int32_t m_spd_direction;
} spdTypeDef;

typedef struct {
	float m_accz;
	float m_gyro;
} balanceDataTypeDef;

//�ɼ�ƽ�⻷��������
void getBalanceData(balanceDataTypeDef* data);

//ƽ�⻷ռ�ձ�
int32_t balanceControl(angleTypeDef* angle);

// �������˲�����
void kalmanFilter(const balanceDataTypeDef* data, angleTypeDef* outAngle);

//ʹ��ռ�ձȿ��Ƶ��
void motorControl(const spdTypeDef* spd);

int limit(int x, int lmt);

#endif
