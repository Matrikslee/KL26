
#ifndef __APP_H__
#define __APP_H__

#include "stdint.h"
extern const float balancedAngle;
typedef struct {
	float m_angle;
	float m_rate;
} angleTypeDef;

typedef struct {
	int32_t leftDuty;
	int32_t rightDuty;
} dutyTypeDef;

typedef struct {
	float m_accz;
	float m_gyro;
} balanceDataTypeDef;

typedef struct {
	float m_leftSpeed;
	float m_rightSpeed;
	float m_avgeSpeed;
} speedDataTypeDef;

//计算平衡环占空比
void balanceCtrl(dutyTypeDef* output);

//计算速度环占空比
void speedCtrl(dutyTypeDef* output);

//计算方向环占空比
void directionCtrl(dutyTypeDef* output);

// 卡尔曼滤波函数
void kalmanFilter(const balanceDataTypeDef* data, angleTypeDef* outAngle);

//使用占空比控制电机
void motorControl(const dutyTypeDef* output);

#endif
