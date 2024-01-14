#ifndef _ECAT_SPI_H_
#define _ECAT_SPI_H_

#include "main.h"

#define DESELECT_SPI HAL_GPIO_WritePin(ECAT_SPI_NSS_GPIO_Port,ECAT_SPI_NSS_Pin,GPIO_PIN_SET);
#define SELECT_SPI HAL_GPIO_WritePin(ECAT_SPI_NSS_GPIO_Port,ECAT_SPI_NSS_Pin,GPIO_PIN_RESET);

uint8_t WR_CMD (uint8_t cmd);

#endif