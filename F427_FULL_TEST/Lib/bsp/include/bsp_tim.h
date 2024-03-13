#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "motor_msg.h"
#include "unitreeA1_cmd.h"

#define BASIC_TIM               TIM6
#define BASIC_TIM_CLK           RCC_APB1Periph_TIM6

#define BASIC_TIM_IRQn          TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler    TIM6_DAC_IRQHandler

void TIMx_Configuration();



#endif