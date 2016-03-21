#ifndef _TPM_H_
#define _TPM_H_

#include "sys.h"
#include "gpio.h"

#define TPM_0                       0
#define TPM_1                       1
#define TPM_2                       2

#define CH0                      0
#define CH1                      1
#define CH2                      2
#define CH3                      3
#define CH4                      4
#define CH5                      5
#define CH6                      6
#define CH7                      7

#define PTE24               0     // FTM0_CH0
#define PTE25               1     // FTM0_CH1
#define PTE29               2     // FTM0_CH2
#define PTE30               3     // FTM0_CH3
#define PTE31               4     // FTM0_CH4
#define PTA0                5     // FTM0_CH5

#define PTA3                6     // FTM0_CH0
#define PTA4                7     // FTM0_CH1
#define PTA5                8     // FTM0_CH2
#define PTC8                9     // FTM0_CH4
#define PTC9               10     // FTM0_CH5



#define PTA20               11     // FTM1_CH0 
#define PTA21               12     // FTM1_CH1
#define PTA12               13     // FTM1_CH0
#define PTA13               14     // FTM1_CH1
#define PTB0                15     // FTM1_CH0
#define PTB1                16     // FTM1_CH1

#define PTE22               17     // FTM2_CH0
#define PTE23               18     // FTM2_CH1

#define PTA1                19    // FTM2_CH0
#define PTA2                20    // FTM2_CH1

#define PTB2                21    // FTM2_CH0
#define PTB3                22    // FTM2_CH1

#define PTB18               23    // FTM2_CH0
#define PTB19               24    // FTM2_CH1


#define DIV1         0
#define DIV2         1
#define DIV4         2
#define DIV8         3
#define DIV16        4
#define DIV32        5
#define DIV64        6
#define DIV128       7





//API
void PWMInit(uint8_t Pin, uint8_t Div, uint16_t modValue);

void PWMOutput(uint8_t Pin, uint16_t Duty);



#endif
