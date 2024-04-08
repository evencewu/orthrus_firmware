#ifndef __NRST_H__
#define __NRST_H__

#include "stm32f4xx_conf.h"

#define ESC_RCC_APB1PERIPH_GPIOX_RSTN   RCC_AHB1Periph_GPIOA
#define ESC_GPIOX_RSTN                  GPIOA
#define ESC_GPIO_Pin_RSTN               GPIO_Pin_2

void rst_setup(void);
void rst_high(void);
void rst_low(void);

#endif