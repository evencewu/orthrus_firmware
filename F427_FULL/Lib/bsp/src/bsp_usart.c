#include "bsp_usart.h"

uint8_t usart6TxBuffer[34];
uint8_t usart6RxBuffer[78];

void USART6_GPIO_Init()
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
}

void USART6_DMA_Init()
{
    DMA_InitTypeDef DMA_InitStructure;

    /*����DMAʱ��*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    /* ��λ��ʼ��DMA������ */
    DMA_DeInit(DMA2_Stream6);

    /* ȷ��DMA��������λ��� */
    while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE)
    {
    }

    DMA_InitStructure.DMA_Channel = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((u32)&USART6->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)usart6TxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = sizeof(usart6TxBuffer);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream6, ENABLE);

    while (DMA_GetCmdStatus(DMA2_Stream6) != ENABLE)
    {
    }
}

void USART6_Init()
{
    USART6_GPIO_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 4800000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &USART_InitStructure);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART6, ENABLE);
}

void USART6_SendData(USART_TypeDef *pUSARTx, uint8_t *data, int length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        USART_SendData(USART6, *(data + i)); // 向串▣发送数据
        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
            ;
    }

    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
        ;
}

void Usart_init()
{
    USART6_Init();
    USART6_DMA_Init();
}

/*****************  ����һ���ַ� **********************/
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
    /* ����һ���ֽ����ݵ�USART */
    USART_SendData(pUSARTx, ch);

    /* �ȴ��������ݼĴ���Ϊ�� */
    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
        ;
}

void Usart_SendData(USART_TypeDef *pUSARTx, uint8_t *data, uint8_t length)
{
    for (int i = 0; i <= length; i++)
    {
        Usart_SendByte(pUSARTx, *(data + i));
    }
}