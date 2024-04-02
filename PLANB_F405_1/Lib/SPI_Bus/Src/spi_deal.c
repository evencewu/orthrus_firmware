#include "main.h"
#include "spi.h"
#include "unitreeA1_msg.h"
#include "unitreeA1_recv.h"
#include "unitreeA1_cmd.h"
#include "data_def.h"
#include "string.h"
#include "spi_deal.h"

#define RXBufsize 20
#define RealRxBufsize 36

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

A1PackageSpiTx A1date[4][3];
A1PackageSpiRx RXA1cmd;

extern motor_recv_t data_motor[4][3];
extern motor_recv_t data_leg[4];
extern motor_send_t cmd_leg[4];

static uint8_t SPI2RXBuf[2][RXBufsize];


void send_A1msgToEcat(int leg_id)
{
    uint8_t txdate[16];
    memcpy(txdate ,&A1date[leg_id][data_leg[leg_id].motor_recv_data.head.motorID],16);
    HAL_SPI_Transmit_DMA(&hspi2,txdate,16);
}

void EcatChat_Init (void)
{
    RC_spi_Init(SPI2RXBuf[0],SPI2RXBuf[1],RXBufsize);
}

void SPI2_IRQHandler(void)
{

  //HAL_SPI_IRQHandler(&hspi2);

    if (SPI2->SR & SPI_FLAG_RXNE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_SPI_CLEAR_FREFLAG(&hspi2);

        if ((hdma_spi2_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_spi2_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RealRxBufsize - hdma_spi2_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_spi2_rx.Instance->NDTR = RealRxBufsize;

            // set memory buffer 1
            // 设定缓冲区1
            hdma_spi2_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == RXBufsize)
            {
                sbus_to_rc_spi(SPI2RXBuf[0],&RXA1cmd);
                DateCheck_DateModfy(&RXA1cmd);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_spi2_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = RealRxBufsize - hdma_spi2_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_spi2_rx.Instance->NDTR = RealRxBufsize;

            // set memory buffer 0
            // 设定缓冲区0
            DMA1_Stream3->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_spi2_rx);

            if (this_time_rx_len == RXBufsize)
            {
                // 处理数据
                sbus_to_rc_spi(SPI2RXBuf[1],&RXA1cmd);
                DateCheck_DateModfy(&RXA1cmd);
            }
        }
    }
}


static void sbus_to_rc_spi(volatile const uint8_t *sbus_buf, A1PackageSpiRx *modf_buf)
{
    if (sbus_buf == NULL || modf_buf == NULL)
    {
        return;
    }
    modf_buf->start[0] = sbus_buf[0];
    modf_buf->start[1] = sbus_buf[1];
    modf_buf->motorID = sbus_buf[2];
    modf_buf->LegID = sbus_buf[3];
    modf_buf->mode = sbus_buf[4];
    modf_buf->T = sbus_buf[5] << 8 | sbus_buf[6];
    modf_buf->W = sbus_buf[7] << 8 | sbus_buf[8];
    modf_buf->Pos = sbus_buf[9] << 24 | sbus_buf[10] << 16 | sbus_buf[11] << 8 | sbus_buf[12];
    modf_buf->K_P = sbus_buf[13] << 8 | sbus_buf[14];
    modf_buf->K_W = sbus_buf[15] << 8 | sbus_buf[16];
    modf_buf->SumCheck = sbus_buf[17] << 24 | sbus_buf[18] << 16 | sbus_buf[19] << 8 | sbus_buf[20];

    if (sbus_buf[0] == 0x01)
    {
        HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
    }
}



void DateCheck_DateModfy (A1PackageSpiRx *modf_buf)
{
    if(modf_buf->SumCheck == modf_buf->start[0] + modf_buf->start[1] + modf_buf->LegID + modf_buf->motorID +modf_buf->mode + modf_buf->T + modf_buf->W + modf_buf->Pos + modf_buf->K_P + modf_buf->K_W)
    {
        if (modf_buf->LegID == 0)
        {
        uint8_t leg_id = 0;
        cmd_leg[leg_id].id = modf_buf->motorID;
        cmd_leg[leg_id].mode = modf_buf->mode;
        cmd_leg[leg_id].K_P = modf_buf->K_P;
        cmd_leg[leg_id].K_W = modf_buf->K_W;
        cmd_leg[leg_id].Pos = modf_buf->Pos;
        cmd_leg[leg_id].W = modf_buf->W;
        cmd_leg[leg_id].T = modf_buf->T;
        }
        if (modf_buf->LegID == 1)
        {
        uint8_t leg_id = 1;
        cmd_leg[leg_id].id = modf_buf->motorID;
        cmd_leg[leg_id].mode = modf_buf->mode;
        cmd_leg[leg_id].K_P = modf_buf->K_P;
        cmd_leg[leg_id].K_W = modf_buf->K_W;
        cmd_leg[leg_id].Pos = modf_buf->Pos;
        cmd_leg[leg_id].W = modf_buf->W;
        cmd_leg[leg_id].T = modf_buf->T;
        }

        if (modf_buf->LegID == 2)
        {
        uint8_t leg_id = 2;
        cmd_leg[leg_id].id = modf_buf->motorID;
        cmd_leg[leg_id].mode = modf_buf->mode;
        cmd_leg[leg_id].K_P = modf_buf->K_P;
        cmd_leg[leg_id].K_W = modf_buf->K_W;
        cmd_leg[leg_id].Pos = modf_buf->Pos;
        cmd_leg[leg_id].W = modf_buf->W;
        cmd_leg[leg_id].T = modf_buf->T;
        }

        if (modf_buf->LegID == 3)
        {
        uint8_t leg_id = 3;
        cmd_leg[leg_id].id = modf_buf->motorID;
        cmd_leg[leg_id].mode = modf_buf->mode;
        cmd_leg[leg_id].K_P = modf_buf->K_P;
        cmd_leg[leg_id].K_W = modf_buf->K_W;
        cmd_leg[leg_id].Pos = modf_buf->Pos;
        cmd_leg[leg_id].W = modf_buf->W;
        cmd_leg[leg_id].T = modf_buf->T;
        }

    }

}
    