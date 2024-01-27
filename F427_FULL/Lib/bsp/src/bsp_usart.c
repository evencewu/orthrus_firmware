#include "bsp_usart.h"
#include "unitreeA1_cmd.h"

#define USART6_DMA_RX_BUFFER_MAX_LENGTH (255)
#define USART6_DMA_TX_BUFFER_MAX_LENGTH (255)

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

void USART6_TX_DMA_Init()
{

    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // DMA2时钟使能
    DMA_DeInit(DMA2_Stream6);
    while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE)
        ;                                                                   // 等待DMA可配置
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;                          // DMA通道配置
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;       // DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6TxBuffer;       // 发送缓存指针
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 // DMA传输方向：内存--->外设
    DMA_InitStructure.DMA_BufferSize = sizeof(usart6TxBuffer);              // 数据传输字节数量
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设非增量模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 存储器增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据长度:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 存储器数据长度:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 使用普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   // 中等优先级 DMA_Priority_High
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;         // 存储器突发单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; // 外设突发单次传输
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);                         // 初始化DMA Stream
    // DMA_Cmd(DMA2_Stream6, DISABLE);                                     // 开启DMA传输
}

void USART6_RX_DMA_Init()
{
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_DeInit(DMA2_Stream2);

    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE)
    {
    }

    /* Configure DMA Stream */
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;                          // DMA请求发出通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;       // 配置外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6RxBuffer;       // 配置存储器地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 传输方向配置
    DMA_InitStructure.DMA_BufferSize = sizeof(usart6RxBuffer);              // 传输大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // memory地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设地址数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // memory地址数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMA模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     // 优先级：高
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // FIFO 模式不使能.
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;           // FIFO 阈值选择
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             // 存储器突发模式选择，可选单次模式、 4 节拍的增量突发模式、 8 节拍的增量突发模式或 16 节拍的增量突发模式。
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     // 外设突发模式选择，可选单次模式、 4 节拍的增量突发模式、 8 节拍的增量突发模式或 16 节拍的增量突发模式。
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    /* DMA Stream enable */
    DMA_Cmd(DMA2_Stream2, ENABLE);
}

void USART6_Init()
{
    USART6_GPIO_Init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 4800000; // 4800000
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &USART_InitStructure);

    //USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); // 串口空闲中断

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitTypeDef NVIC_InitStructure;
    //NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);				//禁止USART1接收不为空中断
    // USART_ITConfig(USART6, USART_IT_TXE, DISABLE);				//禁止USART1发送空中断
    // USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);				//开启USART1空闲中断
    // USART_ITConfig(USART6, USART_IT_TC, ENABLE);				//开启USART1传输完成中断

    // USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

    USART_Cmd(USART6, ENABLE);

    USART_ClearFlag(USART6, USART_FLAG_TC);
    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
        ;
    USART_ClearFlag(USART6, USART_FLAG_TC);
}

void USART6_SendData(uint8_t *data, int length)
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

void USART6_DMA_SendData(uint8_t *data, int length)
{
    if (length < USART6_DMA_TX_BUFFER_MAX_LENGTH)
    {
        memcpy(usart6TxBuffer, data, length);
        DMA_Cmd(DMA2_Stream6, DISABLE); // 关闭DMA传输
        // USART_DMACmd(USART6, USART_DMAReq_Tx, DISABLE);
        while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE)
            ;                                         // 确保DMA可以被设置
        DMA_SetCurrDataCounter(DMA2_Stream6, length); // 数据传输量
        // USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
        DMA_Cmd(DMA2_Stream6, ENABLE); // 开启DMA传输
    }
}

void Usart_init()
{
    USART6_Init();
    USART6_TX_DMA_Init();
    USART6_RX_DMA_Init();
}

//----------------------------------------------------------
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
//----------------------------------------------------------

void USART6_IRQHandler(void)
{
    
    //if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) // 接收中断
    //{
    //    USART_ClearFlag(USART6, USART_IT_RXNE);
    //}

    uint8_t temp;
    if (USART_GetFlagStatus(USART6, USART_FLAG_IDLE) == SET)
    {
        DMA_Cmd(DMA2_Stream2, DISABLE);
        USART_DMACmd(USART6, USART_DMAReq_Rx, DISABLE);
        unitreeA1_rx(3);
        USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
        DMA_Cmd(DMA2_Stream2, ENABLE);

        USART_ClearFlag(USART6, USART_FLAG_IDLE);
        temp = USART6->SR;
        temp = USART6->DR; // 清USART_IT_IDLE标志
    }
}
// u8 c;

// if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)//接收中断
//{
// USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//
// LED2 = !LED2;
// c = USART_ReceiveData(USART2);//读取接收寄存器,读数据会清除中断
// write_loop_2_buf(c);
// }

void DMA2_Stream2_IRQHandler(void)
{
    /* Test on DMA Stream Transfer Complete interrupt */
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
    {
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
    }
}
