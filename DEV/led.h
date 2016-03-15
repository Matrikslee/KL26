#ifndef __LED_H__
#define __LED_H__

#include "gpio.h"
#include "sys.h"

void ledInit(GPIO_Type *GPIO,uint16_t Pin);
void twinkleLed(GPIO_Type *GPIO,uint16_t Pin);


#endif



