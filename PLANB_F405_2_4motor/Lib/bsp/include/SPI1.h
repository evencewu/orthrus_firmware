#ifndef _TOUCHPANEL_H_
#define _TOUCHPANEL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


  #define SPIx                           SPI1
  #define SPIx_CLK                       RCC_APB2Periph_SPI1
  #define SPIx_CLK_INIT                  RCC_APB2PeriphClockCmd

  #define SPIx_SCK_PIN                   GPIO_Pin_5
  #define SPIx_SCK_GPIO_PORT             GPIOA
  #define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOA
  #define SPIx_SCK_SOURCE                GPIO_PinSource5
  #define SPIx_SCK_AF                    GPIO_AF_SPI1

  #define SPIx_MISO_PIN                  GPIO_Pin_6
  #define SPIx_MISO_GPIO_PORT            GPIOA
  #define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOA
  #define SPIx_MISO_SOURCE               GPIO_PinSource6
  #define SPIx_MISO_AF                   GPIO_AF_SPI1

  #define SPIx_MOSI_PIN                  GPIO_Pin_7
  #define SPIx_MOSI_GPIO_PORT            GPIOA
  #define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOA
  #define SPIx_MOSI_SOURCE               GPIO_PinSource7
  #define SPIx_MOSI_AF                   GPIO_AF_SPI1

	#define DESELECT_SPI GPIO_SetBits(GPIOA,GPIO_Pin_4) 
	#define SELECT_SPI GPIO_ResetBits(GPIOA,GPIO_Pin_4) 




/* Private function prototypes -----------------------------------------------*/				
void SPI1_GPIO_Init(void);	
uint8_t WR_CMD (uint8_t cmd)  ;
void ADC_GPIO_Configuration(void);
void ADC_Configuration(void);
void NVIC_Configuration(void);
void TIM_Configuration(uint8_t period)	;
void EXTI0_Configuration(void);
void EXTI1_Configuration(void);
void EXTI8_Configuration(void);
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


