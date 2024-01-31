#include "bsp_usart.h"
#include "unitreeA1_cmd.h"

#define USART6_DMA_RX_BUFFER_MAX_LENGTH (255)
#define USART6_DMA_TX_BUFFER_MAX_LENGTH (255)

uint8_t usart6_rx_num = 0;
uint8_t usart6_rx_flag = 0;
uint8_t usart6TxBuffer[34];
uint8_t usart6RxBuffer[78];

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

    /*定义DMA初始化结构体*/
    DMA_InitTypeDef DMA_InitStructure;
    /*定义中断结构体*/
    NVIC_InitTypeDef NVIC_InitStructure;
    /*使能DMA外设时钟*/
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
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 设置DMA的外设递增模式，外设地址寄存器不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 设置DMA的内存递增模式，内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据字长，数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据字长，数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 设置DMA的传输模式，工作在正常模式，即满了不再接收，而不是循环储存
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 设置DMA的优先级别，非常高的优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       // 指定了FIFO阈值水平
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             // 指定的Burst转移配置内存传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     // 指定的Burst转移配置外围转移
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;         // 通道设置为串口中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 中断占先等级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 中断响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 打开中断
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART6, USART_IT_RXNE, DISABLE); // RX NO Empty，RX非空，RX有数据中断。开启串口接受中断，使用DMA接收时要失能这个，使能空闲中断
    //	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);	//TX Empty，TX为空，发送寄存器DR清零。发送寄存器空闲中断，发送完一个字节后，必须关闭这个中断，否则只要寄存器为空值，就会反复进入中断
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); // 开启串口空闲中断

    /* DMA Stream enable */
    DMA_Cmd(DMA2_Stream2, ENABLE);
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE); // 采用DMA方式接收
}

void USART6_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    // GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    // PC6 PC7
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // usart
    USART_InitStructure.USART_BaudRate = 4800000; // 4800000
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &USART_InitStructure);

    USART_Cmd(USART6, ENABLE);
    USART_ClearFlag(USART6, USART_FLAG_TC);
    while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET)
        ;
    USART_ClearFlag(USART6, USART_FLAG_TC);

    // 开启接受中断
    // USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
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

void Usart_init()
{
    USART6_Init();
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
uint8_t len = 0;
//----------------------------------------------------------
void USART6_IRQHandler(void)
{
    uint8_t data;
/*
    if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
    {
        data = USART_ReceiveData(USART6);
        
        if(usart6_rx_flag == 0)
        {
            data = USART_ReceiveData(USART6);
            if(data == 0xFE)
            {
                usart6_rx_flag = 1;
            }
        }
        else if(usart6_rx_flag == 1)
        {
            data = USART_ReceiveData(USART6);
            if(data == 0xEE)
            {
                usart6_rx_flag = 2;
            }
        }
        else
        {
            usart6RxBuffer[0] = 0xFE;
            usart6RxBuffer[1] = 0xEE;

            usart6RxBuffer[usart6_rx_num+2] = USART_ReceiveData(USART6);

            usart6_rx_num++;
            if(usart6_rx_num >= 76)
            {
                usart6_rx_flag = 0;
                usart6_rx_num = 0;
            }
        }
        

        // usart6RxBuffer
        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    }
*/
    if (USART_GetITStatus(USART6, USART_IT_IDLE) == SET)
    {
        //USART_ClearFlag(USART6, USART_FLAG_IDLE);
        USART6->SR;
        USART6->DR; // 清USART_IT_IDLE标志

        // 关闭DMA
        DMA_Cmd(DMA2_Stream2, DISABLE);
        // 清除标志位
        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        // 获得接收帧帧长

        //unitreeA1_rx(3);

        // memset(buf,0,UART2_DMA_Rx_LEN);
        // memcpy(buf,UART2_DMA_Rx_Buff,len);

        // 设置传输数据长度
        DMA_SetCurrDataCounter(DMA2_Stream2, 78);
        // 打开DMA
        DMA_Cmd(DMA2_Stream2, ENABLE);
        
    }
}
