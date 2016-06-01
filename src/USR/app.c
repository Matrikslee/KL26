#include "app.h"

float getSpeedData(void) {
	const float BMQ_SPEED_RATIO = 0.554508;
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

float getDirectionData(){
	const uint8_t queue_length = 6;
	const uint8_t count_number = 5;
	const uint8_t inductance_number = 4;
	const uint8_t inductance_index[] = {INDUCTANCE_LL,INDUCTANCE_LR,INDUCTANCE_RL,INDUCTANCE_RR};
	static float queue[inductance_number][queue_length];
	static float tmp_value[inductance_number][count_number];
	static float result[inductance_number] = {0};
	static uint8_t que_count = 0;
	float left, right, err, sum;
 	uint8_t i, j;
	for(i = 0; i < inductance_number; ++i){
		for(j = 0; j < count_number; ++j){
			tmp_value[i][j] = getInductanceValue(inductance_index[i]);
		}
	}
	for(i = 0; i < inductance_number; i++) {
		float max_value =0, min_value = 0, sum_value = 0;
		for(j = 0; j < count_number; j++) {
			sum_value += tmp_value[i][j];
			max_value = fmaxf(max_value, tmp_value[i][j]);
			min_value = fminf(min_value, tmp_value[i][j]);
 		}
		queue[i][que_count] = (sum_value-max_value-min_value)/3;
 	}
	que_count = (que_count+1)%queue_length;
	for(i = 0; i < inductance_number; ++i){
		float sum_value = 0;
		for(j = 0; j < queue_length; ++j){
			sum_value += queue[i][j];
		}
		result[i] = sum_value / queue_length;
	}
	
	left  = (result[0]/* + result[3]*/)*0.8;
	right = (result[3]/* + result[1]*/);
	
	err = left - right;
	sum = left + right + 1;
	
	return 100*err/sum;
}

int32_t balanceCtrl() {
	const float dt = 0.005;
	const float ratio = 0.99611;
	const float balance_Kp = 1100;
  const float balance_Kd = 20;
	const float set_angle = -5.5;
	static float cur_angle = 0;
	float err_angle;
	float accz;
	float gyro;
	float result;

	accz = getAcczValue(ACCZ_Y);
	gyro = getGyroValue(GYRO_Y);
	
	cur_angle = ratio*(cur_angle+gyro*dt)+(1-ratio)*accz;
	
	err_angle = cur_angle - set_angle;
	
	result = balance_Kp*err_angle + balance_Kd*gyro;
	
	return (int32_t) result;
}

#define RUN_SPEED (35)

float speedCalc(float m_speed) {
	const float max_speed_i = 1100;
	const float speedCtrlKp = 240;
	const float speedCtrlKi = 5;
	static float speed_err, speed_p, speed_i = 0;
	
	speed_err =  m_speed - RUN_SPEED/(time>500?1:5);
	
	speed_p = speed_err*speedCtrlKp;
	speed_i += speed_err*speedCtrlKi;
	
	speed_i = flimit(speed_i, max_speed_i);
	
	return speed_p + speed_i;
}

int32_t speedCtrl() {
	const uint8_t maxSpeed_period = 100;
	static uint8_t speed_period = 0;
	static float cur_speed = 0, pre_speed = 0, err_speed, result;
	float m_speed;
	
	m_speed = getSpeedData();
	
	if(!speed_period) {
		pre_speed = cur_speed;
		cur_speed = speedCalc(m_speed);
	}
	speed_period = (speed_period+1)%maxSpeed_period;
	
	err_speed = cur_speed - pre_speed;
	
	result = err_speed*(speed_period*1./maxSpeed_period) + pre_speed;
	
	return (int32_t) result;
}

float directionCalc(){
	const float gyro_K = 0;
	const float sensor_Kp = 8;
	const float sensor_Kd = 270;
	static float cur_sensor = 0, pre_sensor = 0;
	static float gyro;
	static float sensor_p;
	static float sensor_d;
	
	cur_sensor = getDirectionData();
	gyro = getGyroValue(GYRO_X);
	
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

#define MAX_TURN_DUTY 500

void motorControl(int32_t balance, int32_t speed, int32_t turn){
	static int32_t tmp, left, right;
	
	turn = limit(turn, HALF_MAX_PWM_DUTY);
	
	tmp = HALF_MAX_PWM_DUTY + limit(balance+speed,HALF_MAX_PWM_DUTY-MAX_TURN_DUTY);
	
	left  = tmp-turn;
	right = tmp+turn;
	
	PWMOutput(PWM_LEFT, left);
	PWMOutput(PWM_RIGHT,right);
}
