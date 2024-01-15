#include "ecat_spi.h"

extern SPI_HandleTypeDef hspi2;

uint8_t WR_CMD(uint8_t cmd)
{
    uint8_t temp;

    HAL_SPI_TransmitReceive(&hspi2, &cmd, &temp, 1, 100);

    return temp;
}