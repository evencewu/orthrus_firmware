#include "main.h"
#include "unitreeA1_cmd.h"
#include "bsp_rc_it.h"
#include "bsp_dma.h"
#include <string.h>
#include <stdio.h>

extern motor_recv_t data_leg[4];
extern motor_recv_t data_motor[4][3];

A1_buf a1_buf_RB, a1_buf_RF, a1_buf_LB, a1_buf_LF;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf);

static uint8_t sbus_rx1_buf[2][SBUS_RX_BUF_NUM]; // 128�ֽ�,��Խ��
static uint8_t sbus_rx2_buf[2][SBUS_RX_BUF_NUM]; // 128�ֽ�,��Խ��
static uint8_t sbus_rx3_buf[2][SBUS_RX_BUF_NUM]; // 128�ֽ�,��Խ��
static uint8_t sbus_rx6_buf[2][SBUS_RX_BUF_NUM]; // 128�ֽ�,��Խ��

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

// static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf)
//{
//		a1_buf->motorID = sbus_buf[2];
//		a1_buf->MError  = sbus_buf[7];
//		a1_buf->T       = sbus_buf[12] << 8 | sbus_buf[13];
//		a1_buf->Pos     = sbus_buf[30] << 24 | sbus_buf[31] << 16 | sbus_buf[32] << 8 | sbus_buf[33];
// }
