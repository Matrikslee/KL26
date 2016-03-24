
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"

typedef struct {
	float m_angle;
	float m_rate;
} angleTypeDef;

typedef struct {
	uint32_t leftDuty;
	uint32_t rightDuty;
} dutyTypeDef;

typedef struct {
	float m_accz;
	float m_gyro;
} balanceDataTypeDef;

typedef struct {
	float m_Left;
	float m_Right;
} speedDataTypeDef;


typedef struct {
	uint32_t m_value[4];
	int32_t m_dir_flag;
} directionDataTypeDef;

//�ɼ�ƽ�⻷��������
void getBalanceData(balanceDataTypeDef* data);

void getDirectionData(directionDataTypeDef* data);
//�ɼ�����������
void getSpeedData(speedDataTypeDef* data);
	
//����ƽ�⻷ռ�ձ�
void balanceCtrl(angleTypeDef* angle, dutyTypeDef* output);

//���㷽��ռ�ձ�
void directionCtrl(directionDataTypeDef* data, dutyTypeDef* output);

// �������˲�����
void kalmanFilter(const balanceDataTypeDef* data, angleTypeDef* outAngle);

//ʹ��ռ�ձȿ��Ƶ��
void motorControl(const dutyTypeDef* output);

int limit(int x, int lmt);

#endif
