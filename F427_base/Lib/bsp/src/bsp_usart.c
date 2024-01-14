#include "bsp_usart.h"

motor_send_t cmd_motor[4]; 
motor_recv_t data_motor[4];    
motor_recv_t motor_data[12];

static void usart6_setup()
{

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    RCC_APB2PeriphClockCmd(USART6_APBPeriph_RCC, ENABLE);
    RCC_AHB1PeriphClockCmd(USART6_GPIO_TX_RCC, ENABLE);
    RCC_AHB1PeriphClockCmd(USART6_GPIO_RX_RCC, ENABLE);

    USART_InitStructure.USART_BaudRate = 4800000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART6, &USART_InitStructure);

    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

    USART_Cmd(USART6, ENABLE);

    GPIO_InitStructure.GPIO_Pin = USART6_GPIO_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART6_TX_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(USART6_TX_PORT, USART6_TX_PIN_SOURCE, USART6_GPIO_AF);

    GPIO_InitStructure.GPIO_Pin = USART6_GPIO_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART6_RX_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(USART6_RX_PORT, USART6_RX_PIN_SOURCE, USART6_GPIO_AF);

    USART_GetFlagStatus(USART6, USART_FLAG_TC); // 清除TXE，防止第一个数据没有发送出来
}

static void Usart6_DmaTxConfig()
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(USART6_TX_DMA_RCC, ENABLE);
    DMA_DeInit(USART6_TX_DMA_STREAM);
    while (DMA_GetCmdStatus(USART6_TX_DMA_STREAM) != DISABLE)
    {
    }

    DMA_InitStructure.DMA_Channel = USART6_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&cmd_motor[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = sizeof(cmd_motor[0]);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(USART6_TX_DMA_STREAM, &DMA_InitStructure);
    DMA_ITConfig(USART6_TX_DMA_STREAM, DMA_IT_TC, ENABLE); // 此处必须开启中断，否则DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6)一直为0
    DMA_Cmd(USART6_TX_DMA_STREAM, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE); // 使能DMA发送
}

// DMA TX RX配置模板函数

static void Usart6_DmaRxConfig()
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(USART6_GPIO_RX_RCC, ENABLE);
    DMA_DeInit(USART6_RX_DMA_STREAM);

    while (DMA_GetCmdStatus(USART6_RX_DMA_STREAM) != DISABLE)
    {
    }

    DMA_InitStructure.DMA_Channel = USART6_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR /*((uint32_t)usart->usart_base+0x04)usart->usart_base->DR*/;
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&data_motor[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

    DMA_InitStructure.DMA_BufferSize = sizeof(data_motor[0]);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(USART6_RX_DMA_STREAM, &DMA_InitStructure);

    DMA_Cmd(USART6_RX_DMA_STREAM, ENABLE);

    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
}

void init_usart(void)
{
    usart6_setup();
    Usart6_DmaTxConfig();
    Usart6_DmaRxConfig();
}