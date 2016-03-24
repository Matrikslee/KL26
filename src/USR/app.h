
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
	float m_leftSpeed;
	float m_rightSpeed;
	float m_totalSpeed;
	uint8_t m_leftFlag;
	uint8_t m_rightFlag;
} speedTypeDef;

typedef struct {
	uint32_t m_value[4];
	int32_t m_dir_flag;
} directionDataTypeDef;


//get balance data
void getBalanceData(balanceDataTypeDef* data);

//get speed data
void getSpeedData(speedTypeDef* data);

//get direction data
void getDirectionData(directionDataTypeDef* data);

//计算平衡环占空比
void balanceCtrl(angleTypeDef* angle, dutyTypeDef* output);

//计算速度环占空比
void speedCtrl(speedTypeDef* speed, dutyTypeDef* output);

//计算方向环占空比
void directionCtrl(directionDataTypeDef* data, dutyTypeDef* output);

// 卡尔曼滤波函数
void kalmanFilter(const balanceDataTypeDef* data, angleTypeDef* outAngle);

//使用占空比控制电机
void motorControl(const dutyTypeDef* output);

int limit(int x, int lmt);

#endif
