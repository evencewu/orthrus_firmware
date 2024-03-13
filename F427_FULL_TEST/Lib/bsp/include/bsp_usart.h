#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "stm32f4xx_conf.h"

#include "stm32f4xx.h"
#include <string.h>
#include <stdio.h>

#define SBUS_RX_BUF_NUM 128u

#define RC_FRAME_LENGTH 78u

typedef __packed struct
{
    uint8_t motorID;
	uint8_t MError;
	uint16_t T;
	uint16_t W;
	uint32_t Pos;

} A1_buf;

void remote_control_init(void);

void unitreeA1_rx(int leg_id);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf);
const A1_buf *get_remote_control_point(void);

extern void RC1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC2_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC3_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void SendData(USART_TypeDef* pUSARTx,uint8_t *data, int length);



#endif
