#include "main.h"
#include "RC.h"
#include "unitreeA1_cmd.h"
#include "bsp_dma.h"
#include "tim.h"
#include "bsp_can.h"

#define L 0
#define R 1

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern motor_recv_t data_leg[2];
extern motor_recv_t data_motor[2][3];

Leg leg[2];

static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf);

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM]; // 128字节,防越界

void remote_control_init(void)
{
    RC1_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    RC6_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    extern uint8_t usart1RxBuffer[78];
    if (huart1.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                uint8_t byte[8];
                sbus_to_rc(sbus_rx_buf[0], &leg[L].a1_buf);
                unitreeA1_rx(0);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 处理数据
                uint8_t byte[8];
                sbus_to_rc(sbus_rx_buf[1], &leg[L].a1_buf);
                unitreeA1_rx(0);
            }
        }
    }
}

void USART6_IRQHandler(void)
{
    extern uint8_t usart6RxBuffer[78];
    if (huart6.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
    }
    else if (USART6->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart6);

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                uint8_t byte[8];
                sbus_to_rc(sbus_rx_buf[0], &leg[R].a1_buf);
                unitreeA1_rx(1);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 0
            // 设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 处理数据
                uint8_t byte[8];
                sbus_to_rc(sbus_rx_buf[1], &leg[R].a1_buf);
                unitreeA1_rx(1);
            }
        }
    }
}

void unitreeA1_rx(int leg_id)
{
    data_leg[leg_id].motor_recv_data.head.motorID = leg[leg_id].a1_buf.motorID;
    data_leg[leg_id].motor_recv_data.Mdata.MError = leg[leg_id].a1_buf.MError;
    data_leg[leg_id].motor_recv_data.Mdata.T = leg[leg_id].a1_buf.T;
    data_leg[leg_id].motor_recv_data.Mdata.W = leg[leg_id].a1_buf.W;
    data_leg[leg_id].motor_recv_data.Mdata.Acc = leg[leg_id].a1_buf.Acc;
    data_leg[leg_id].motor_recv_data.Mdata.Pos = leg[leg_id].a1_buf.Pos;

    data_leg[leg_id].motor_id = data_leg[leg_id].motor_recv_data.head.motorID;
    data_leg[leg_id].MError = data_leg[leg_id].motor_recv_data.Mdata.MError;
    data_leg[leg_id].T = (float)((int32_t)data_leg[leg_id].motor_recv_data.Mdata.T / 256.0f);
    data_leg[leg_id].W = (float)((int32_t)data_leg[leg_id].motor_recv_data.Mdata.W / 128.0f);
    data_leg[leg_id].Acc = (float)data_leg[leg_id].motor_recv_data.Mdata.Acc;
    data_leg[leg_id].Pos = (float)((data_leg[leg_id].motor_recv_data.Mdata.Pos / 16384.0f) * 6.2832f);

    data_motor[leg_id][data_leg[leg_id].motor_id].motor_id = data_leg[leg_id].motor_id;
    data_motor[leg_id][data_leg[leg_id].motor_id].MError = data_leg[leg_id].MError;
    data_motor[leg_id][data_leg[leg_id].motor_id].T = data_leg[leg_id].T;
    data_motor[leg_id][data_leg[leg_id].motor_id].W = data_leg[leg_id].W;
    data_motor[leg_id][data_leg[leg_id].motor_id].Acc = data_leg[leg_id].Acc;
    data_motor[leg_id][data_leg[leg_id].motor_id].Pos = data_leg[leg_id].Pos;
}

// dma接收赋值函数
static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf)
{
    if (sbus_buf == NULL || a1_buf == NULL)
    {
        return;
    }

    a1_buf->motorID = sbus_buf[2];
    a1_buf->MError = sbus_buf[7];
    a1_buf->T = sbus_buf[13] << 8 | sbus_buf[12];
    a1_buf->W = sbus_buf[15] << 8 | sbus_buf[14];
    a1_buf->Acc = sbus_buf[28] << 8 | sbus_buf[27];
    a1_buf->Pos = sbus_buf[33] << 24 | sbus_buf[32] << 16 | sbus_buf[31] << 8 | sbus_buf[30];
}

// 电机发送数据缓冲存储区域
void sum_cheak()
{
    for (int leg_id = 0; leg_id < 2; leg_id++)
    {
        for (int motor_id = 0; motor_id < 3; motor_id++)
        {
            leg[leg_id].motor_can[motor_id].sum_cheak_temp = (uint32_t)leg[leg_id].motor_can[motor_id].cmd_leg_temp.Pos | (uint32_t)leg[leg_id].motor_can[motor_id].cmd_leg_temp.K_P | (uint32_t)leg[leg_id].motor_can[motor_id].cmd_leg_temp.K_W | (uint32_t)leg[leg_id].motor_can[motor_id].cmd_leg_temp.mode | (uint32_t)leg[leg_id].motor_can[motor_id].cmd_leg_temp.T | (uint32_t)leg[leg_id].motor_can[motor_id].cmd_leg_temp.W;

            if (leg[leg_id].motor_can[motor_id].Checksum == leg[leg_id].motor_can[motor_id].sum_cheak_temp)
            {
                leg[leg_id].motor_can[motor_id].cmd_leg_confirm.mode = leg[leg_id].motor_can[motor_id].cmd_leg_temp.mode;
                leg[leg_id].motor_can[motor_id].cmd_leg_confirm.K_P = (float)leg[leg_id].motor_can[motor_id].cmd_leg_temp.K_P;
                leg[leg_id].motor_can[motor_id].cmd_leg_confirm.K_W = (float)leg[leg_id].motor_can[motor_id].cmd_leg_temp.K_W;
                leg[leg_id].motor_can[motor_id].cmd_leg_confirm.Pos = (float)leg[leg_id].motor_can[motor_id].cmd_leg_temp.Pos;
                leg[leg_id].motor_can[motor_id].cmd_leg_confirm.T = (float)leg[leg_id].motor_can[motor_id].cmd_leg_temp.T;
                leg[leg_id].motor_can[motor_id].cmd_leg_confirm.W = (float)leg[leg_id].motor_can[motor_id].cmd_leg_temp.W;
            }
        }
    }
}

// 设定电机发送数据结构体
void SetMotorMsg(int motor_id)
{
    for (int leg_id = 0; leg_id < 2; leg_id++)
    {
        cmd_leg[leg_id].id = motor_id;
        cmd_leg[leg_id].mode = leg[leg_id].motor_can[motor_id].cmd_leg_confirm.mode;
        cmd_leg[leg_id].K_P = leg[leg_id].motor_can[motor_id].cmd_leg_confirm.K_P;
        cmd_leg[leg_id].K_W = leg[leg_id].motor_can[motor_id].cmd_leg_confirm.K_W;
        cmd_leg[leg_id].Pos = leg[leg_id].motor_can[motor_id].cmd_leg_confirm.Pos;
        cmd_leg[leg_id].T = leg[leg_id].motor_can[motor_id].cmd_leg_confirm.T;
        cmd_leg[leg_id].W = leg[leg_id].motor_can[motor_id].cmd_leg_confirm.W;
    }
}

int usart_motor_flag = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // tim4中断回调函数
{
    if (htim->Instance == TIM4)
    {
        if (usart_motor_flag > 2)
        {
            usart_motor_flag = 0;
        }
        sum_cheak();

        SetMotorMsg(usart_motor_flag);
        unitreeA1_tx(L);
        unitreeA1_tx(R);
        usart_motor_flag++;
    }
}
