#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32f4xx_conf.h"

#include "motor_msg.h"

extern motor_send_t cmd_motor[4]; 
extern motor_recv_t data_motor[4];    
extern motor_recv_t motor_data[12];

/******************************USART6***********************************/

#define USART6_GPIO_TX GPIO_Pin_6
#define USART6_TX_PIN_SOURCE GPIO_PinSource6
#define USART6_GPIO_RX GPIO_Pin_7
#define USART6_RX_PIN_SOURCE GPIO_PinSource7
#define USART6_TX_PORT GPIOC
#define USART6_RX_PORT GPIOC
#define USART6_GPIO_TX_RCC RCC_AHB1Periph_GPIOC
#define USART6_GPIO_RX_RCC RCC_AHB1Periph_GPIOC
#define USART6_APBPeriph_RCC RCC_APB2Periph_USART6
#define USART6_GPIO_AF GPIO_AF_USART6
#define USART6_RX_DMA_RCC RCC_AHB1Periph_DMA2
#define USART6_RX_DMA_STREAM DMA2_Stream1
#define USART6_RX_DMA_CHANNEL DMA_Channel_5

#define USART6_TX_DMA_RCC RCC_AHB1Periph_DMA2
#define USART6_TX_DMA_STREAM DMA2_Stream6
#define USART6_TX_DMA_CHANNEL DMA_Channel_5
#define USART6_DMA_IT_TCIF DMA_IT_TCIF6

void init_usart();

static void usart6_setup();
static void Usart6_DmaTxConfig();
static void Usart6_DmaRxConfig();


//void HAL_UART_Transmit_DMA();
//void HAL_UART_Receive_DMA();

#endif
