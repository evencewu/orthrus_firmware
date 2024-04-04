#include "unitreeA1_recv.h"

#include "data_def.h"

#define LF 0
#define LB 1
#define RF 2
#define RB 3

Leg leg[4];

motor_recv_t data_motor[4][3];
motor_recv_t data_leg[4];
extern A1PackageSpiTx A1date[4][3];

static uint8_t sbus_rx_buf_LF[2][SBUS_RX_BUF_NUM]; // 128字节,防越界
static uint8_t sbus_rx_buf_LB[2][SBUS_RX_BUF_NUM]; // 128字节,防越界
static uint8_t sbus_rx_buf_RF[2][SBUS_RX_BUF_NUM]; // 128字节,防越界
static uint8_t sbus_rx_buf_RB[2][SBUS_RX_BUF_NUM]; // 128字节,防越界

void remote_control_init(void)
{
    RC1_init(sbus_rx_buf_LF[0], sbus_rx_buf_LF[1], SBUS_RX_BUF_NUM);
    RC2_init(sbus_rx_buf_LB[0], sbus_rx_buf_LB[1], SBUS_RX_BUF_NUM);
    RC3_init(sbus_rx_buf_RB[0], sbus_rx_buf_RB[1], SBUS_RX_BUF_NUM);
    RC6_init(sbus_rx_buf_RF[0], sbus_rx_buf_RF[1], SBUS_RX_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    
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
                sbus_to_rc(sbus_rx_buf_LF[0], &leg[LF].a1_buf);
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
                sbus_to_rc(sbus_rx_buf_LF[1], &leg[LF].a1_buf);
                unitreeA1_rx(0);
                
            }
        }
    }
}

void USART2_IRQHandler(void)
{
   
    if (huart2.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart2);
    }
    else if (USART2->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart2);

        if ((hdma_usart2_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart2_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart2_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart2_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart2_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf_LB[0], &leg[LB].a1_buf);
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
                sbus_to_rc(sbus_rx_buf_LB[1], &leg[LB].a1_buf);
                unitreeA1_rx(1);
                
            }
        }
    }
}

void USART3_IRQHandler(void)
{
  
    if (huart3.Instance->SR & UART_FLAG_RXNE) // 接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf_RB[0], &leg[RB].a1_buf);
                unitreeA1_rx(3);
                
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
                sbus_to_rc(sbus_rx_buf_RB[1], &leg[RB].a1_buf);
                unitreeA1_rx(3);
               
            }
        }
    }
}

void USART6_IRQHandler(void)
{
   
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
                sbus_to_rc(sbus_rx_buf_RF[0], &leg[RF].a1_buf);
                unitreeA1_rx(2);
               
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
                sbus_to_rc(sbus_rx_buf_RF[1], &leg[RF].a1_buf);
                unitreeA1_rx(2);
               
            }
        }
    }
}

void unitreeA1_rx(int leg_id)
{
    data_leg[leg_id].motor_recv_data.head.motorID = leg[leg_id].a1_buf.motorID;
    data_leg[leg_id].motor_recv_data.Mdata.MError = leg[leg_id].a1_buf.MError;
    data_leg[leg_id].motor_recv_data.Mdata.mode = leg[leg_id].a1_buf.mode;
    data_leg[leg_id].motor_recv_data.Mdata.Temp = leg[leg_id].a1_buf.Temp;
    data_leg[leg_id].motor_recv_data.Mdata.MError = leg[leg_id].a1_buf.MError;
    data_leg[leg_id].motor_recv_data.Mdata.T = leg[leg_id].a1_buf.T;
    data_leg[leg_id].motor_recv_data.Mdata.W = leg[leg_id].a1_buf.W;
    data_leg[leg_id].motor_recv_data.Mdata.Acc = leg[leg_id].a1_buf.Acc;
    data_leg[leg_id].motor_recv_data.Mdata.Pos = leg[leg_id].a1_buf.Pos;



    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].start[0] = 0xD2; 
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].start[1] = 0xFE;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].LegID = leg_id;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].MotorID = data_leg[leg_id].motor_recv_data.head.motorID;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].MError = data_leg[leg_id].motor_recv_data.Mdata.MError;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].mode = data_leg[leg_id].motor_recv_data.Mdata.mode;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].Temp = data_leg[leg_id].motor_recv_data.Mdata.Temp;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].T = data_leg[leg_id].motor_recv_data.Mdata.T;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].W = data_leg[leg_id].motor_recv_data.Mdata.W;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].Acc = data_leg[leg_id].motor_recv_data.Mdata.Acc;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].Pos = data_leg[leg_id].motor_recv_data.Mdata.Pos;
    A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID].SumCheck = 0xff + 0xfe + leg_id +data_leg[leg_id].motor_recv_data.head.motorID +data_leg[leg_id].motor_recv_data.Mdata.mode + data_leg[leg_id].motor_recv_data.Mdata.Temp+data_leg[leg_id].motor_recv_data.Mdata.T + data_leg[leg_id].motor_recv_data.Mdata.W + data_leg[leg_id].motor_recv_data.Mdata.Acc + data_leg[leg_id].motor_recv_data.Mdata.Pos;


/*
    data_leg[leg_id].motor_id = data_leg[leg_id].motor_recv_data.head.motorID;
    data_leg[leg_id].MError = data_leg[leg_id].motor_recv_data.Mdata.MError;
    data_leg[leg_id].T = data_leg[leg_id].motor_recv_data.Mdata.T;
    data_leg[leg_id].W = data_leg[leg_id].motor_recv_data.Mdata.W;
    data_leg[leg_id].Acc = data_leg[leg_id].motor_recv_data.Mdata.Acc;
    data_leg[leg_id].Pos = data_leg[leg_id].motor_recv_data.Mdata.Pos;

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
*/
}

// dma接收赋值函数
static void sbus_to_rc(volatile const uint8_t *sbus_buf, A1_buf *a1_buf)
{
    if (sbus_buf == NULL || a1_buf == NULL)
    {
        return;
    }

    a1_buf->motorID = sbus_buf[2];
    a1_buf->mode = sbus_buf[4];
    a1_buf->Temp = sbus_buf[6];
    a1_buf->MError = sbus_buf[7];
    a1_buf->T = sbus_buf[13] << 8 | sbus_buf[12];
    a1_buf->W = sbus_buf[15] << 8 | sbus_buf[14];
    a1_buf->Acc = sbus_buf[28] << 8 | sbus_buf[27];
    a1_buf->Pos = sbus_buf[33] << 24 | sbus_buf[32] << 16 | sbus_buf[31] << 8 | sbus_buf[30];
}
/*
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

*/
