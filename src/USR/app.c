#include "app.h"

//get speed data
float getSpeedData(void) {
	static const float BMQ_SPEED_RATIO = 0.554508;
	static uint8_t leftFlag, rightFlag;
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
	static const uint8_t queue_length = 6;
	static const uint8_t count_number = 5;
	static const uint8_t inductance_number = 4;
	static const uint8_t inductance_index[] = {INDUCTANCE_LL,INDUCTANCE_LR,INDUCTANCE_RL,INDUCTANCE_RR};
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
	right = result[0]; //+ value[1];
	left = result[3]; //+ value[3];
	
	err = left - right;
	sum = left + right + 1; // sum > 0
	
	return 100*err/sum;
}
//calculate the balance data
int32_t balanceCtrl() {
	static const float dt = 0.005;
	static const float ratio = 0.995;
	static const float balance_Kp = 1300;
	static const float balance_Kd = 20;
	static const float set_angle = -5;  //1.5
	static float cur_angle = 0;
	static float err_angle;
	static float accz;
	static float gyro;
	float result;

	accz = getIMUValue(ACCZ_Y);
	
	accz = asinhf(accz);
	gyro = getIMUValue(GYRO_Y);
	
	cur_angle = ratio*(cur_angle+gyro*dt)+(1-ratio)*accz;
	
	err_angle = cur_angle - set_angle;
	
	result = balance_Kp*err_angle + balance_Kd*gyro;
	
	return (int32_t) result;
}

#define RUN_SPEED 35

int32_t get_set_speed(){
	static const uint8_t max_cnt = 3;
	static uint8_t cnt = 0;
	if(cnt < max_cnt) { ++cnt; }
	return RUN_SPEED*cnt/max_cnt;
}

float speedCalc(int32_t m_speed) {
	static const float max_speed_i = 1300;
	static const float speedCtrlKp = 350;
	static const float speedCtrlKi = 10;
	static float speed_err, speed_p, speed_i = 0;
	
	speed_err =  m_speed - get_set_speed();
	
	speed_p = speed_err*speedCtrlKp;
	speed_i += speed_err*speedCtrlKi;
	
	speed_i = flimit(speed_i, max_speed_i);
	
	return speed_p + speed_i;
}

//calculate the speed data
int32_t speedCtrl() {
	static const uint8_t maxSpeed_period = 100;
	static uint8_t speed_period = 0;
	static float cur_speed = 0, pre_speed = 0, err_speed, result;
	static int32_t m_speed;
	
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
	static const float gyro_K = 1;
	static const float sensor_Kp = 3;
	static const float sensor_Kd = 300;
	static float cur_sensor = 0, pre_sensor = 0;
	static float gyro;
	static float sensor_p;
	static float sensor_d;
	
	cur_sensor = getDirectionData();
	gyro = getIMUValue(GYRO_X);
	
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

#define MAX_TURN_DUTY 600

//control the motors by using the SPD data
void motorControl(int32_t balance, int32_t speed, int32_t turn){
	static int32_t tmp, left, right;
	
	turn = limit(turn, HALF_MAX_PWM_DUTY);
	
	tmp = HALF_MAX_PWM_DUTY + limit(balance+speed,HALF_MAX_PWM_DUTY-MAX_TURN_DUTY);
	
	left  = tmp-turn;
	right = tmp+turn;
	
	PWMOutput(PWM_LEFT, left);
	PWMOutput(PWM_RIGHT,right);
}
