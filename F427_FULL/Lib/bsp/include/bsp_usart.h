#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32f4xx_conf.h"

#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

extern uint8_t usart6TxBuffer[34];
extern uint8_t usart6RxBuffer[78];

void USART6_GPIO_Init();
void USART6_Init();
void USART6_DMA_Init();
void USART6_SendData(uint8_t *data, int length);
void USART6_DMA_SendData(uint8_t *data, int length);

void Usart_init();

void Debug_USART_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);

#endif
