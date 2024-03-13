#include "bsp_usart.h"
#include "unitreeA1_cmd.h"

#include <string.h>
#include <stdio.h>

extern motor_recv_t data_leg[4];
extern motor_recv_t data_motor[4][3];

A1_buf a1_buf_RB, a1_buf_RF, a1_buf_LB, a1_buf_LF;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf);

static uint8_t sbus_rx1_buf[2][SBUS_RX_BUF_NUM];
static uint8_t sbus_rx2_buf[2][SBUS_RX_BUF_NUM];
static uint8_t sbus_rx3_buf[2][SBUS_RX_BUF_NUM];
static uint8_t sbus_rx6_buf[2][SBUS_RX_BUF_NUM];

const A1_buf *get_remote_control_point(void)
{
    return &a1_buf_RF;
}

void remote_control_init(void)
{
    RC1_init(sbus_rx1_buf[0], sbus_rx1_buf[1], SBUS_RX_BUF_NUM);
    RC2_init(sbus_rx2_buf[0], sbus_rx2_buf[1], SBUS_RX_BUF_NUM);
    RC3_init(sbus_rx3_buf[0], sbus_rx3_buf[1], SBUS_RX_BUF_NUM);
    RC6_init(sbus_rx6_buf[0], sbus_rx6_buf[1], SBUS_RX_BUF_NUM);
}

void unitreeA1_rx(int leg_id)
{
    switch (leg_id)
    {
    case 0:
        data_leg[leg_id].motor_recv_data.head.motorID = a1_buf_LF.motorID;
        data_leg[leg_id].motor_recv_data.Mdata.MError = a1_buf_LF.MError;
        data_leg[leg_id].motor_recv_data.Mdata.T = a1_buf_LF.T;
        data_leg[leg_id].motor_recv_data.Mdata.W = a1_buf_LF.W;
        data_leg[leg_id].motor_recv_data.Mdata.Pos = a1_buf_LF.Pos;
        break;

    case 1:
        data_leg[leg_id].motor_recv_data.head.motorID = a1_buf_LB.motorID;
        data_leg[leg_id].motor_recv_data.Mdata.MError = a1_buf_LB.MError;
        data_leg[leg_id].motor_recv_data.Mdata.T = a1_buf_LB.T;
        data_leg[leg_id].motor_recv_data.Mdata.W = a1_buf_LB.W;
        data_leg[leg_id].motor_recv_data.Mdata.Pos = a1_buf_LB.Pos;
        break;

    case 2:
        data_leg[leg_id].motor_recv_data.head.motorID = a1_buf_RF.motorID;
        data_leg[leg_id].motor_recv_data.Mdata.MError = a1_buf_RF.MError;
        data_leg[leg_id].motor_recv_data.Mdata.T = a1_buf_RF.T;
        data_leg[leg_id].motor_recv_data.Mdata.W = a1_buf_RF.W;
        data_leg[leg_id].motor_recv_data.Mdata.Pos = a1_buf_RF.Pos;
        break;

    case 3:
        data_leg[leg_id].motor_recv_data.head.motorID = a1_buf_RB.motorID;
        data_leg[leg_id].motor_recv_data.Mdata.MError = a1_buf_RB.MError;
        data_leg[leg_id].motor_recv_data.Mdata.T = a1_buf_RB.T;
        data_leg[leg_id].motor_recv_data.Mdata.W = a1_buf_RB.W;
        data_leg[leg_id].motor_recv_data.Mdata.Pos = a1_buf_RB.Pos;
        break;
    }

    data_leg[leg_id].motor_id = data_leg[leg_id].motor_recv_data.head.motorID;
    data_leg[leg_id].MError = data_leg[leg_id].motor_recv_data.Mdata.MError;
    data_leg[leg_id].T = (float)((int32_t)data_leg[leg_id].motor_recv_data.Mdata.T / 256.0f);
    data_leg[leg_id].W = (float)((int32_t)data_leg[leg_id].motor_recv_data.Mdata.W / 128.0f);
    data_leg[leg_id].Acc = data_leg[leg_id].motor_recv_data.Mdata.Acc;
    data_leg[leg_id].Pos = (float)((data_leg[leg_id].motor_recv_data.Mdata.Pos / 16384.0f) * 6.2832f);

    data_motor[leg_id][data_leg[leg_id].motor_id].motor_id = data_leg[leg_id].motor_id;
    data_motor[leg_id][data_leg[leg_id].motor_id].MError = data_leg[leg_id].MError;
    data_motor[leg_id][data_leg[leg_id].motor_id].T = data_leg[leg_id].T;
    data_motor[leg_id][data_leg[leg_id].motor_id].W = data_leg[leg_id].W;
    data_motor[leg_id][data_leg[leg_id].motor_id].Pos = data_leg[leg_id].Pos;
}

void USART1_IRQHandler(void)
{
    extern uint8_t usart1RxBuffer[78];
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // ���յ�����
    {
        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������

        temp = USART1->SR;
        temp = USART1->DR;
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;

        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������
        temp = USART1->SR;
        temp = USART1->DR;

        if ((DMA2_Stream2->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // ʧЧDMA
            DMA_Cmd(DMA2_Stream2, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream2->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA2_Stream2->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // �趨������1
            DMA2_Stream2->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA2_Stream2, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx1_buf[0], &a1_buf_LF);
                unitreeA1_rx(0);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // ʧЧDMA
            DMA_Cmd(DMA2_Stream2, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream2->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA2_Stream2->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // �趨������0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA2_Stream2, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // ��������
                sbus_to_rc(sbus_rx1_buf[1], &a1_buf_LF);
                unitreeA1_rx(0);
            }
        }
    }
}

void USART2_IRQHandler(void)
{
    extern uint8_t usart2RxBuffer[78];
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // ���յ�����
    {

        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������

        temp = USART2->SR;
        temp = USART2->DR;
    }
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;

        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������
        temp = USART2->SR;
        temp = USART2->DR;

        if ((DMA1_Stream5->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // ʧЧDMA
            DMA_Cmd(DMA1_Stream5, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA1_Stream5->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA1_Stream5->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // �趨������1
            DMA1_Stream5->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA1_Stream5, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx2_buf[0], &a1_buf_RB);
                unitreeA1_rx(3);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // ʧЧDMA
            DMA_Cmd(DMA1_Stream5, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA1_Stream5->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA1_Stream5->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // �趨������0
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA1_Stream5, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // ��������
                sbus_to_rc(sbus_rx2_buf[1], &a1_buf_RB);
                unitreeA1_rx(3);
            }
        }
    }
}

void USART3_IRQHandler(void)
{
    extern uint8_t usart3RxBuffer[78];
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // ���յ�����
    {

        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������

        temp = USART3->SR;
        temp = USART3->DR;
    }
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;

        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������
        temp = USART3->SR;
        temp = USART3->DR;

        if ((DMA1_Stream1->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // ʧЧDMA
            DMA_Cmd(DMA1_Stream1, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA1_Stream1->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA1_Stream1->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // �趨������1
            DMA1_Stream1->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA1_Stream1, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx3_buf[0], &a1_buf_LB);
                unitreeA1_rx(1);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // ʧЧDMA
            DMA_Cmd(DMA1_Stream1, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA1_Stream1->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA1_Stream1->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // �趨������0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA1_Stream1, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // ��������
                sbus_to_rc(sbus_rx3_buf[1], &a1_buf_LB);
                unitreeA1_rx(1);
            }
        }
    }
}

void USART6_IRQHandler(void)
{
    extern uint8_t usart6RxBuffer[78];
    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        volatile uint32_t temp;
        temp = USART6->SR;
        temp = USART6->DR;
    }
    else if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        volatile uint32_t temp;
        // ���PE��־��ͨ����ȡSR�Ĵ������ȡDR�Ĵ�����˳����������
        temp = USART6->SR;
        temp = USART6->DR;

        if ((DMA2_Stream1->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // ʧЧDMA

            DMA_Cmd(DMA2_Stream1, DISABLE);

            uint16_t dma_counter = USART6->DR; // �۲���

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream1->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA2_Stream1->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // �趨������1
            DMA2_Stream1->CR |= DMA_SxCR_CT;

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA2_Stream1, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx6_buf[0], &a1_buf_RF);
                unitreeA1_rx(2);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // ʧЧDMA

            DMA_Cmd(DMA2_Stream1, DISABLE);

            // get receive data length, length = set_data_length - remain_length
            // ��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA2_Stream1->NDTR;

            // reset set_data_lenght
            // �����趨���ݳ���
            DMA2_Stream1->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // �趨������0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // ʹ��DMA
            DMA_Cmd(DMA2_Stream1, ENABLE);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // ��������
                sbus_to_rc(sbus_rx6_buf[1], &a1_buf_RF);
                unitreeA1_rx(2);
            }
        }
    }
}
//
static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf)
{
    if (sbus_buf == 0 || a1_buf == 0)
    {
        return;
    }
    //	uint8_t testdate[78] = {0};
    //	uint32_t crc_test = sbus_buf[74]<< 24| sbus_buf[75] << 16 | sbus_buf[76] << 8|sbus_buf[77];

    //		memcpy(testdate,&sbus_buf,74);
    //	if (crc32_core((uint32_t *)(&testdate),19) == crc_test)
    //	{
    a1_buf->motorID = sbus_buf[2];
    a1_buf->MError = sbus_buf[7];
    a1_buf->T = sbus_buf[13] << 8 | sbus_buf[12];
    a1_buf->W = sbus_buf[15] << 8 | sbus_buf[14];
    a1_buf->Pos = sbus_buf[33] << 24 | sbus_buf[32] << 16 | sbus_buf[31] << 8 | sbus_buf[30];
    //	}
}

void RC1_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    // GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    // PC6 PC7
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    // usart
	//USART_OverSampling8Cmd(USART1, ENABLE);
    USART_InitStructure.USART_BaudRate = 4800000; // 4800000
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
	//USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);


    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         // 通道设置为串口中�?
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 中断占先等级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 中断响应优先�?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 打开中断
    NVIC_Init(&NVIC_InitStructure);
	

	//DMA	
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_DeInit(DMA2_Stream2);

	 while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE)
	{
	 }

	/* Configure DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // DMA请求发出通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 传输方向配置
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 设置DMA的外设递增模式，外设地址寄存器不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 设置DMA的内存递增模式，内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 设置DMA的传输模式，工作在正常模式，即满了不再接收，而不是循环储�?
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 设置DMA的优先级别，非常高的优先�?
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 指定如果FIFO模式或直接模式将用于指定的流 �? 不使能FIFO模式
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    //SET_BIT(USART1->CR3, USART_CR3_DMAR);
   // USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

    //disable DMA
    //ʧЧDMA
    //__HAL_DMA_DISABLE(&hdma_usart1_rx);
	DMA_Cmd(DMA2_Stream2, DISABLE);
	
    while(DMA2_Stream2->CR & DMA_SxCR_EN)
    {
        DMA_Cmd(DMA2_Stream2, DISABLE);
    }

    DMA2_Stream2->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //�ڴ滺����1
    DMA2_Stream2->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    DMA2_Stream2->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    DMA2_Stream2->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(DMA2_Stream2->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    DMA_Cmd(DMA2_Stream2, ENABLE);
	USART_Cmd(USART1, ENABLE);

}

void RC2_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    // GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    // PC6 PC7
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    // usart
	USART_OverSampling8Cmd(USART2, ENABLE);
    USART_InitStructure.USART_BaudRate = 4800000; // 4800000
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
	//USART_OverSampling8Cmd(USART2, ENABLE);
	//USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);


    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;         // 通道设置为串口中�?
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 中断占先等级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 中断响应优先�?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 打开中断
    NVIC_Init(&NVIC_InitStructure);
	

	//DMA	
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Stream5);

	 while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE)
	{
	 }

	/* Configure DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // DMA请求发出通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 传输方向配置
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 设置DMA的外设递增模式，外设地址寄存器不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 设置DMA的内存递增模式，内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 设置DMA的传输模式，工作在正常模式，即满了不再接收，而不是循环储�?
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 设置DMA的优先级别，非常高的优先�?
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 指定如果FIFO模式或直接模式将用于指定的流 �? 不使能FIFO模式
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    //SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    //disable DMA
    //ʧЧDMA
    DMA_Cmd(DMA1_Stream5, DISABLE);
    while(DMA1_Stream5->CR & DMA_SxCR_EN)
    {
		DMA_Cmd(DMA1_Stream5, DISABLE);
    }

    DMA1_Stream5->PAR = (uint32_t) & (USART2->DR);
    //memory buffer 1
    //�ڴ滺����1
    DMA1_Stream5->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    DMA1_Stream5->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    DMA1_Stream5->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(DMA1_Stream5->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
	DMA_Cmd(DMA1_Stream5, ENABLE);
	USART_Cmd(USART2, ENABLE);

}


void RC3_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    // GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    // PC6 PC7
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    // usart
	USART_OverSampling8Cmd(USART3, ENABLE);
    USART_InitStructure.USART_BaudRate = 4800000; // 4800000
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
	//USART_OverSampling8Cmd(USART3, ENABLE);
	//USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);


    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;         // 通道设置为串口中�?
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 中断占先等级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 中断响应优先�?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 打开中断
    NVIC_Init(&NVIC_InitStructure);
	

	//DMA	
	DMA_InitTypeDef DMA_InitStructure;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Stream1);

	 while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE)
	{
	 }

	/* Configure DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // DMA请求发出通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 传输方向配置
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 设置DMA的外设递增模式，外设地址寄存器不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 设置DMA的内存递增模式，内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 设置DMA的传输模式，工作在正常模式，即满了不再接收，而不是循环储�?
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 设置DMA的优先级别，非常高的优先�?
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 指定如果FIFO模式或直接模式将用于指定的流 �? 不使能FIFO模式
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);

	//DMA_Cmd(DMA2_Stream1, ENABLE);
	 
	 
	//˫������
	
	
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    //SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);


    //disable DMA
    //ʧЧDMA
    DMA_Cmd(DMA1_Stream1, DISABLE);
    while(DMA1_Stream1->CR & DMA_SxCR_EN)
    {
        DMA_Cmd(DMA1_Stream1, DISABLE);
    }

    DMA1_Stream1->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //�ڴ滺����1
    DMA1_Stream1->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    DMA1_Stream1->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    DMA1_Stream1->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(DMA1_Stream1->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
	DMA_Cmd(DMA1_Stream1, ENABLE);
	USART_Cmd(USART3, ENABLE);
}


void RC6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
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
	//USART_OverSampling8Cmd(USART6, ENABLE);
    USART_InitStructure.USART_BaudRate = 4800000; // 4800000
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART6, &USART_InitStructure);
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);


    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;         // 通道设置为串口中�?
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 中断占先等级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 中断响应优先�?
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 打开中断
    NVIC_Init(&NVIC_InitStructure);
	

	//DMA	
	DMA_InitTypeDef DMA_InitStructure;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_DeInit(DMA2_Stream1);

	 while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE)
	{
	 }

	/* Configure DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;                          // DMA请求发出通道
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 传输方向配置
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 设置DMA的外设递增模式，外设地址寄存器不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 设置DMA的内存递增模式，内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据字长，数据宽度为8�?
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 设置DMA的传输模式，工作在正常模式，即满了不再接收，而不是循环储�?
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 设置DMA的优先级别，非常高的优先�?
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 指定如果FIFO模式或直接模式将用于指定的流 �? 不使能FIFO模式
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	//DMA_Cmd(DMA2_Stream1, ENABLE);
	 
	 
	//˫������
	 
	//enable the DMA transfer for the receiver request
	//ʹ��DMA���ڽ���
	//SET_BIT(USART6->CR3, USART_CR3_DMAR);
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

	//enalbe idle interrupt
	//ʹ�ܿ����ж�
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

	//disable DMA
	//ʧЧDMA
	DMA_Cmd(DMA2_Stream1, DISABLE);
	while(DMA2_Stream1->CR & DMA_SxCR_EN)
	{
		DMA_Cmd(DMA2_Stream1, DISABLE);
	}

	DMA2_Stream1->PAR = (uint32_t) & (USART6->DR);
	//memory buffer 1
	//�ڴ滺����1
	DMA2_Stream1->M0AR = (uint32_t)(rx1_buf);
	//memory buffer 2
	//�ڴ滺����2
	DMA2_Stream1->M1AR = (uint32_t)(rx2_buf);
	//data length
	//���ݳ���
	DMA2_Stream1->NDTR = dma_buf_num;
	//enable double memory buffer
	//ʹ��˫������
	SET_BIT(DMA2_Stream1->CR, DMA_SxCR_DBM);

	//enable DMA
	//ʹ��DMA
	DMA_Cmd(DMA2_Stream1, ENABLE);
	USART_Cmd(USART6, ENABLE);
	
}

void SendData(USART_TypeDef* pUSARTx,uint8_t *data, int length)
{
 for (uint32_t i = 0; i < length; i++)
    {
        USART_SendData(pUSARTx,*data++); // 閸氭垳瑕嗛埢锝呭絺闁焦鏆熼幑锟�
        while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
    }

    while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}



//----------------------------------------------------------
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
    
    USART_SendData(pUSARTx, ch);

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