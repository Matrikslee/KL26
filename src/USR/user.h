
#ifndef __USR_H__
#define __USR_H__

#include "counter.h"
#include "stdint.h"
#include "gpio.h"
#include "uart.h"
#include "sys.h"
#include "adc.h"
#include "pit.h"
#include "TPM.h"
#include "tools.h"
#include <math.h>
#include <stdint.h>

#define GYRO_X (0)
#define GYRO_Y (1)
#define GYRO_Z (2)

#define ACCZ_X (3)
#define ACCZ_Y (4)
#define ACCZ_Z (5)

#define INDUCTANCE_LL (6)
#define INDUCTANCE_LR (7)
#define INDUCTANCE_RL (8)
#define INDUCTANCE_RR (9)

#define PWM_LEFT  PTA5
#define PWM_RIGHT PTA12

#define MAX_PWM_DUTY (6000)
#define HALF_MAX_PWM_DUTY (MAX_PWM_DUTY/2)

void PWM_userInit(void);
void GPIO_userInit(void);
void PIT_userInit(void);
void IMU_userInit(void);
void inductance_userInit(void);
void gyro_offsetInit(void);

float getAcczValue(uint8_t);
float getGyroValue(uint8_t);
float getInductanceValue(uint8_t);

#endif
