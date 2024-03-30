#ifndef __BSP_SPI2_H__
#define __BSP_SPI2_H__

#include "stm32f4xx.h"

#define SPI2_CLK RCC_APB1Periph_SPI2
#define SPI2_CLK_INIT RCC_APB1PeriphClockCmd

#define SPI2_SCK_PIN GPIO_Pin_10
#define SPI2_SCK_GPIO_PORT GPIOB
#define SPI2_SCK_GPIO_CLK RCC_AHB1Periph_GPIOB
#define SPI2_SCK_SOURCE GPIO_PinSource10
#define SPI2_SCK_AF GPIO_AF_SPI2

#define SPI2_MISO_PIN GPIO_Pin_2
#define SPI2_MISO_GPIO_PORT GPIOC
#define SPI2_MISO_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SPI2_MISO_SOURCE GPIO_PinSource2
#define SPI2_MISO_AF GPIO_AF_SPI2

#define SPI2_MOSI_PIN GPIO_Pin_3
#define SPI2_MOSI_GPIO_PORT GPIOC
#define SPI2_MOSI_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SPI2_MOSI_SOURCE GPIO_PinSource3
#define SPI2_MOSI_AF GPIO_AF_SPI2

//#define DESELECT_SPI GPIO_SetBits(GPIOC, GPIO_Pin_4)
//#define SELECT_SPI GPIO_ResetBits(GPIOC, GPIO_Pin_4)

void SPI2_Init(void);
void SPI2_GPIO_Init(void);

void spi2_w_cmd(uint8_t cmd);
uint8_t spi2_r_cmd();

uint8_t spi2_wr_cmd(uint8_t cmd);

#endif