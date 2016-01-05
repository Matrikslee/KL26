
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"

#define SPD_LMT 200
#define MAX_SPD (6000-SPD_LMT)
#define MIN_SPD (0+SPD_LMT)

typedef struct {
	float angle;
	float angleDot;
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

//采集平衡环所需数据
void getBalanceData(balanceDataTypeDef* data);

//平衡环占空比
int32_t balanceControl(const balanceDataTypeDef* data, angleTypeDef* angle);

// 卡尔曼滤波函数
void kalman(const balanceDataTypeDef* data, angleTypeDef* outAngle);

//使用占空比控制电机
void motorControl(const spdTypeDef* spd);

int limit(int x, int lmt);

#endif
