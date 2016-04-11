#include "app.h"
#include "user.h"
#include "TPM.h"
#include "include.h"
#include "counter.h"
#include "tools.h"

static int32_t deadVoltage_L = 50;
static int32_t deadVoltage_R = 0;
//vertical accz angle =0
const float  Asin_to_Angle[] = {
-90.000000,-81.890386,-78.521659,-75.930132,-73.739795,-71.805128,-70.051556,-68.434815,-66.926082,-65.505352,
-64.158067,-62.873247,-61.642363,-60.458639,-59.316583,-58.211669,-57.140120,-56.098738,-55.084794,-54.095931,
-53.130102,-52.185511,-51.260575,-50.353889,-49.464198,-48.590378,-47.731416,-46.886394,-46.054480,-45.234915,
-44.427004,-43.630109,-42.843643,-42.067065,-41.299873,-40.541602,-39.791819,-39.050123,-38.316134,-37.589503,
-36.869898,-36.157008,-35.450543,-34.750226,-34.055798,-33.367013,-32.683639,-32.005455,-31.332251,-30.663830,
-30.000000,-29.340582,-28.685402,-28.034297,-27.387108,-26.743684,-26.103881,-25.467560,-24.834587,-24.204835,
-23.578178,-22.954499,-22.333683,-21.715617,-21.100196,-20.487315,-19.876874,-19.268775,-18.662925,-18.059230,
-17.457603,-16.857956,-16.260205,-15.664267,-15.070062,-14.477512,-13.886540,-13.297072,-12.709033,-12.122352,
-11.536959,-10.952784,-10.369760,-9.787819, -9.206896, -8.626927, -8.047846, -7.469592, -6.892103, -6.315316,
-5.739170, -5.163607, -4.588566, -4.013987, -3.439813, -2.865984, -2.292443, -1.719131, -1.145992, -0.572967,
0.000000,0.572967, 1.145992, 1.719131, 2.292443, 2.865984, 3.439813, 4.013987, 4.588566, 5.163607,  5.739170,
6.315316, 6.892103, 7.469592, 8.047846, 8.626927, 9.206896, 9.787819, 10.369760,10.952784,11.536959,
12.122352,12.709033,13.297072,13.886540,14.477512,15.070062,15.664267,16.260205,16.857956,17.457603,
18.059230,18.662925,19.268775,19.876874,20.487315,21.100196,21.715617,22.333683,22.954499,23.578178,
24.204835,24.834587,25.467560,26.103881,26.743684,27.387108,28.034297,28.685402,29.340582,30.000000,
30.663830,31.332251,32.005455,32.683639,33.367013,34.055798,34.750226,35.450543,36.157008,36.869898,
37.589503,38.316134,39.050123,39.791819,40.541602,41.299873,42.067065,42.843643,43.630109,44.427004,
45.234915,46.054480,46.886394,47.731416,48.590378,49.464198,50.353889,51.260575,52.185511,53.130102,
54.095931,55.084794,56.098738,57.140120,58.211669,59.316583,60.458639,61.642363,62.873247,64.158067,
65.505352,66.926082,68.434815,70.051556,71.805128,73.739795,75.930132,78.521659,81.890386,90.000000,
};
#define GYRO_ZERO_X  0x083C //static_gyro output
#define GYRO_ZERO_Y  0x0770 //static_gyro output
#define ACCZ_ZERO  2080 //vertical_accz output

//kalman fliter
void kalmanFilter(const balanceDataTypeDef* measureData, angleTypeDef* result){
	const float qAngle=0.001, qGyro=0.003, rAngle=0.5, dt=0.005;    //0.001/0.003/0.67  //0.004/0.008/0.01
	static float e;
	static float qBias;
	static float k0, k1;
	static float t0, t1;
	static float angleErr;
 	static float pCt0, pCt1;
 	static float pDot[4] ={0,0,0,0};
 	static float p[2][2] = {{ 1, 0 },{ 0, 1 }};

 	result->m_angle += (measureData->m_gyro-qBias)*dt;

 	pDot[0] = qAngle - p[0][1] - p[1][0];
 	pDot[1] = -p[1][1];
 	pDot[2] = -p[1][1];
 	pDot[3] = qGyro;

 	p[0][0] += pDot[0] * dt;
 	p[0][1] += pDot[1] * dt;
 	p[1][0] += pDot[2] * dt;
 	p[1][1] += pDot[3] * dt;

 	angleErr = measureData->m_accz - result->m_angle;

 	pCt0 = p[0][0];
 	pCt1 = p[1][0];
 	e = rAngle + pCt0;

 	k0 = pCt0 / e;
 	k1 = pCt1 / e;

 	t0 = pCt0;
 	t1 = p[0][1];

 	p[0][0] -= k0 * t0;
 	p[0][1] -= k0 * t1;
 	p[1][0] -= k1 * t0;
 	p[1][1] -= k1 * t1;

 	result->m_angle	+= k0 * angleErr;
	qBias+= k1 * angleErr;
 	result->m_rate = measureData->m_gyro-qBias;
 }

//get balance data
void getBalanceData(balanceDataTypeDef* data){
	float tmp;
	uint16_t tmp_accz, tmp_gyro;
	//get the value of the accz and process
	tmp_accz = ADC_GetValue(2)>>4;
	tmp = (tmp_accz-ACCZ_ZERO)*0.086;  //0.050708;//0.086
	if(tmp>100) { tmp = 100; }
	if(tmp<-100) { tmp = -100; }       //set the angle limit
	data->m_accz = Asin_to_Angle[(uint8_t)(tmp+100)];
	if(data->m_accz > -2.80)
		GPIO_SetBits(PTC,3);
	else
		GPIO_ResetBits(PTC,3);
	//get the value of the gyro and process
	tmp_gyro = ADC_GetValue(5)>>4;
	data->m_gyro = (tmp_gyro-GYRO_ZERO_Y)*0.120248;	//0.120248  //need to debug to config the value(after kalman the value = 0)
}

const unsigned char directionChannel[] = {7,9,8,6};

//get speed data
float getSpeedData(void) {
	static const float BMQ_SPEED_RATIO = 0.0554508;
	uint8_t leftFlag, rightFlag;
	float leftSpeed, rightSpeed;
	leftFlag = !GPIO_ReadInputDataBit(PTB,9);
	rightFlag = GPIO_ReadInputDataBit(PTB,10);
	leftSpeed = Counter0_Read()*(leftFlag?-1.0f:1.0f);
	rightSpeed = Counter1_Read()*(rightFlag?-1.0f:1.0f);
	Counter0_Clear();
	Counter1_Clear();
	return BMQ_SPEED_RATIO*(leftSpeed + rightSpeed)/2.0f;
}

//get & calc direction data
float getDirectionData(){
	static const uint8_t directionChannel[] = {7,9,8,6};
	static const uint8_t countNumber = 5;
	static const uint8_t maxQueCount = 6;
	static const uint8_t adNumber = 4;
	static float que_value[adNumber][maxQueCount];
	static float tmp_value[adNumber][countNumber];
	static float value[adNumber] = {0};
	static uint8_t que_count = 0;
	float left, right, err, sum;
 	uint8_t i, j;
	for(i = 0; i < adNumber; ++i){
		for(j = 0; j < countNumber; ++j){
			tmp_value[i][j] = ADC_GetValue(directionChannel[i])>>8;
		}
	}
	for(i = 0; i < adNumber; i++) {
		float max_value =0, min_value = 0, sum_value = 0;
		for(j = 0; j < countNumber; j++) {
			sum_value += tmp_value[i][j];
			max_value = max(max_value, tmp_value[i][j]);
			min_value = min(min_value, tmp_value[i][j]);
 		}
		que_value[i][que_count] = (sum_value-max_value-min_value)/3;
 	}
	que_count = (que_count+1)%maxQueCount;
	for(i = 0; i < adNumber; ++i){
		float sum_value = 0;
		for(j = 0; j < maxQueCount; ++j){
			sum_value += que_value[i][j];
		}
		value[i] = sum_value / maxQueCount;
	}
	left = value[0];
	right = value[3];
	
	err = left - right;
	sum = left + right + 1; // sum > 0
	
	return 100*err/sum;
}

const float balancedAngle = -10.3;

//calculate the balance data
int32_t balanceCtrl() {
	static const float balanceKp = 1000;
	static const float balanceKd = 0;
	static angleTypeDef angle;
	int32_t result;
	
	balanceDataTypeDef measure;
	getBalanceData(&measure);
	kalmanFilter(&measure, &angle);
	result = balanceKp*(angle.m_angle - balancedAngle)+balanceKd*angle.m_rate;
	
	return (int32_t) result;
}

float speedCalc(){
	static const float SPEED_TO_DUTY = 12.034;
	static const float maxSpeed_I = 15;
	static const float speedCtrlKp = 2;
	static const float speedCtrlKi = 0.5;
	static const float setSpeed = 80;
	static float speedError, speed_p = 0, speed_i = 0;
	speedError = getSpeedData() - setSpeed;
	
	speed_p = speedError;
	speed_i = limit(speed_i+speedError, maxSpeed_I);
	
	return SPEED_TO_DUTY*(speed_p*speedCtrlKp + speed_i*speedCtrlKi);
}

//calculate the speed data
int32_t speedCtrl() {
	static const uint8_t maxSpeed_period = 60;
	static uint8_t speed_period = 0;
	static float cur_speed = 0, pre_speed = 0, err_speed, result;
	
	if(!speed_period) {
		pre_speed = cur_speed;
		cur_speed = speedCalc();
	}
	speed_period = (speed_period+1)%maxSpeed_period;
	
	err_speed = cur_speed - pre_speed;
	
	result = err_speed*(speed_period*1./maxSpeed_period) + pre_speed;
	
	return (int32_t) result;
}

float getXGyro(){
	static float tmp_gyro;
	tmp_gyro = ADC_GetValue(3)>>4;
	return 0.23578*(tmp_gyro-GYRO_ZERO_X);
}

float directionCalc(){
	static const float gyro_K = 0;
	static const float sensor_Kp = 2.0;
	static const float sensor_Kd = 2.0;
	static float cur_sensor = 0, pre_sensor = 0;
	static float gyro;
	static float sensor_p;
	static float sensor_d;
	
	cur_sensor = getDirectionData();
	gyro = getXGyro();

	sensor_p = cur_sensor;
	sensor_d = cur_sensor - pre_sensor;
	pre_sensor = cur_sensor;
	
	return sensor_p*sensor_Kp + sensor_d*sensor_Kd + gyro*gyro_K;
}

int32_t directionCtrl(){
	static float result;
	
	result = directionCalc();
	
	return (int32_t) result;
}

//control the motors by using the SPD data
void motorControl(int32_t balance, int32_t speed, int32_t turn){
	static int32_t tmp, left, right;
	
	tmp = limit(balance+speed, maxPwmDuty*5/6);
	
	left  =  limit(tmp-turn, maxPwmDuty-200);
	right = limit(tmp+turn, maxPwmDuty-200);
	
	if(left > 0){
		PWMOutput(pwmArray[1], 0);
		PWMOutput(pwmArray[0], deadVoltage_L+left);
	} else {
		PWMOutput(pwmArray[0], 0);
		PWMOutput(pwmArray[1], deadVoltage_L-left);
	}

	if(right > 0){
		PWMOutput(pwmArray[3], 0);
		PWMOutput(pwmArray[2], deadVoltage_R+right);
	} else {
		PWMOutput(pwmArray[2], 0);
		PWMOutput(pwmArray[3], deadVoltage_R-right);
	}
}
