#include "bsp_spi2.h"

#define BUFFER_SIZE 10

uint8_t txBuffer[BUFFER_SIZE];
uint8_t rxBuffer[BUFFER_SIZE];

void SPI2_Init(void)
{
    SPI2_GPIO_Init();

    SPI_InitTypeDef SPI_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // Enable SPI2 and DMA clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    // Configure SPI2
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPI_Cmd(SPI2, ENABLE);

    // Configure DMA for SPI2 RX
    // DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI2->DR);
    // DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxBuffer;
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    // DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    // DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    // DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    // DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    // DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    // DMA_Init(DMA1_Stream3, &DMA_InitStructure);

    // Configure DMA for SPI2 TX
    // DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI2->DR);
    // DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txBuffer;
    // DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    // DMA_Init(DMA1_Stream4, &DMA_InitStructure);

    // Enable DMA interrupts
    // DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
    // DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

    // Enable DMA streams
    // DMA_Cmd(DMA1_Stream3, ENABLE);
    // DMA_Cmd(DMA1_Stream4, ENABLE);

    // Enable SPI2 RX DMA
    // SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable SPI2 TX DMA
    // SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
}

void SPI2_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable the SPI clock */
    SPI2_CLK_INIT(SPI2_CLK, ENABLE);

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(SPI2_SCK_GPIO_CLK | SPI2_MISO_GPIO_CLK | SPI2_MOSI_GPIO_CLK, ENABLE);

    /* Connect SPI pins to AF5 */
    GPIO_PinAFConfig(SPI2_SCK_GPIO_PORT, SPI2_SCK_SOURCE, SPI2_SCK_AF);
    GPIO_PinAFConfig(SPI2_MISO_GPIO_PORT, SPI2_MISO_SOURCE, SPI2_MISO_AF);
    GPIO_PinAFConfig(SPI2_MOSI_GPIO_PORT, SPI2_MOSI_SOURCE, SPI2_MOSI_AF);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN;
    GPIO_Init(SPI2_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* SPI  MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN;
    GPIO_Init(SPI2_MISO_GPIO_PORT, &GPIO_InitStructure);

    /* SPI  MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN;
    GPIO_Init(SPI2_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /* Enable GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
}

uint8_t spi2_wr_cmd(uint8_t cmd)
{
    uint8_t temp;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(SPI2, cmd);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    /* Read SPI1 received data */
    temp = SPI_I2S_ReceiveData(SPI2);
    return temp;
}