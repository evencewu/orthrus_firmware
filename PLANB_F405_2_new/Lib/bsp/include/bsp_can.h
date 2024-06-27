#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f4xx_conf.h"

/*CAN硬件相关的定义*/

#define CAN1_CLK RCC_APB1Periph_CAN1
#define CAN2_CLK RCC_APB1Periph_CAN2

/*接收中断号*/

#define CAN1_RX_IRQ CAN1_RX0_IRQn
#define CAN2_RX_IRQ CAN2_RX0_IRQn

/*接收中断服务函数*/

#define CAN1_RX_PIN GPIO_Pin_11
#define CAN1_TX_PIN GPIO_Pin_12
#define CAN1_TX_GPIO_PORT GPIOA
#define CAN1_RX_GPIO_PORT GPIOA
#define CAN1_TX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define CAN1_RX_GPIO_CLK RCC_AHB1Periph_GPIOA
#define CAN1_AF_PORT GPIO_AF_CAN1
#define CAN1_RX_SOURCE GPIO_PinSource11
#define CAN1_TX_SOURCE GPIO_PinSource12

#define CAN2_RX_PIN GPIO_Pin_5
#define CAN2_TX_PIN GPIO_Pin_6
#define CAN2_TX_GPIO_PORT GPIOB
#define CAN2_RX_GPIO_PORT GPIOB
#define CAN2_TX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define CAN2_RX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define CAN2_AF_PORT GPIO_AF_CAN2
#define CAN2_RX_SOURCE GPIO_PinSource5
#define CAN2_TX_SOURCE GPIO_PinSource6

void CAN1_Config(void);
void CAN2_Config(void);

//void CAN2_LoadBalanceInit(void);



#endif